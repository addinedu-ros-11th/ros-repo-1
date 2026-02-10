#!/bin/bash

# 구조화된 응답 기능 사용 예시 스크립트

echo "========================================"
echo "구조화된 자연어 프롬프트 해석 데모"
echo "========================================"
echo ""

# 색상 정의
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}1. LLM 서버 시작하기${NC}"
echo "다음 명령어로 LLM 서버를 시작하세요:"
echo ""
echo -e "  ${GREEN}python -m ai_server.llm_server${NC}"
echo ""
echo "서버가 시작되면 Enter를 눌러 계속하세요..."
read

echo ""
echo -e "${BLUE}2. 테스트 실행${NC}"
echo ""

# 방법 1: 직접 테스트
echo -e "${YELLOW}[방법 1] 직접 테스트 (gRPC 없이)${NC}"
echo "LLM 서비스만 단독으로 테스트합니다."
echo ""
echo "실행 명령어:"
echo -e "  ${GREEN}python tests/test_structured_response.py${NC}"
echo ""
echo "이 방법을 실행하시겠습니까? (y/n)"
read answer
if [ "$answer" = "y" ]; then
    python tests/test_structured_response.py
fi

echo ""
echo ""

# 방법 2: gRPC 통합 테스트
echo -e "${YELLOW}[방법 2] gRPC 통합 테스트${NC}"
echo "실제 gRPC 통신을 통해 테스트합니다."
echo "주의: LLM 서버가 실행 중이어야 합니다!"
echo ""
echo "실행 명령어:"
echo -e "  ${GREEN}python tests/test_structured_response_grpc.py${NC}"
echo ""
echo "이 방법을 실행하시겠습니까? (y/n)"
read answer
if [ "$answer" = "y" ]; then
    python tests/test_structured_response_grpc.py
fi

echo ""
echo "========================================"
echo "데모 완료!"
echo "========================================"
echo ""
echo "더 자세한 정보는 다음 문서를 참고하세요:"
echo "  - ai_server/docs/STRUCTURED_RESPONSE_GUIDE.md"
echo ""
