#!/bin/bash

# AI Server 테스트 가이드
# 이 스크립트는 테스트 순서를 안내합니다

echo "========================================"
echo "AI Server 테스트 가이드"
echo "========================================"
echo ""

echo "📋 테스트 순서:"
echo ""
echo "1단계: 환경 설정 확인"
echo "  ✓ .env 파일이 생성되었습니다"
echo ""

echo "2단계: AI 서버 실행 (새 터미널에서)"
echo "  옵션 A) 모든 서버 시작:"
echo "    ./ai_server/start_all_servers.sh"
echo ""
echo "  옵션 B) 개별 서버 시작:"
echo "    터미널 1: ./ai_server/start_llm_server.sh"
echo "    터미널 2: ./ai_server/start_vision_server.sh"
echo ""

echo "3단계: 서버 실행 확인 (이 터미널에서)"
echo "  다음 명령으로 서버가 떴는지 확인:"
echo "    netstat -tuln | grep -E '50051|50052'"
echo ""

echo "4단계: 테스트 실행"
echo "  옵션 A) 모든 서버 테스트:"
echo "    python3 ai_server/test_client_separated.py"
echo ""
echo "  옵션 B) LLM만 테스트:"
echo "    python3 ai_server/test_client_separated.py llm"
echo ""
echo "  옵션 C) Vision만 테스트:"
echo "    python3 ai_server/test_client_separated.py vision"
echo ""

echo "========================================"
echo "빠른 테스트 (올인원):"
echo "========================================"
echo ""
echo "터미널 1에서:"
echo "  ./ai_server/start_all_servers.sh"
echo ""
echo "터미널 2에서 (2-3초 대기 후):"
echo "  python3 ai_server/test_client_separated.py"
echo ""
echo "========================================"
