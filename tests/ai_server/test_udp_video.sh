#!/bin/bash

# UDP 영상 스트리밍 통합 테스트 스크립트

echo "========================================"
echo "UDP 영상 스트리밍 테스트"
echo "========================================"
echo ""

# 색상 코드
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 1. 의존성 확인
echo "📦 1단계: 의존성 확인"
python3 -c "import cv2; import numpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ opencv-python, numpy 설치됨${NC}"
else
    echo -e "${YELLOW}⚠ opencv-python, numpy가 필요합니다${NC}"
    echo "설치 명령: pip install opencv-python numpy"
    read -p "지금 설치하시겠습니까? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip install opencv-python numpy
    else
        exit 1
    fi
fi
echo ""

# 2. 포트 확인
echo "🔍 2단계: 포트 상태 확인"
PORT_50052=$(netstat -tuln 2>/dev/null | grep ':50052' | wc -l)
PORT_54321=$(netstat -tuln 2>/dev/null | grep ':54321' | wc -l)

if [ $PORT_50052 -gt 0 ]; then
    echo -e "${YELLOW}⚠ 포트 50052가 이미 사용 중입니다${NC}"
else
    echo -e "${GREEN}✓ 포트 50052 사용 가능${NC}"
fi

if [ $PORT_54321 -gt 0 ]; then
    echo -e "${YELLOW}⚠ 포트 54321이 이미 사용 중입니다${NC}"
    echo "  기존 프로세스를 종료하려면: kill \$(lsof -t -i:54321)"
else
    echo -e "${GREEN}✓ 포트 54321 사용 가능${NC}"
fi
echo ""

# 3. 테스트 옵션 선택
echo "🎥 3단계: 테스트 모드 선택"
echo "  1) 테스트 패턴 (권장)"
echo "  2) 웹캠 (웹캠이 있는 경우)"
echo "  3) 취소"
echo ""
read -p "선택 (1-3): " choice

case $choice in
    1)
        MODE="test"
        echo -e "${GREEN}테스트 패턴 모드 선택${NC}"
        ;;
    2)
        MODE="webcam"
        echo -e "${GREEN}웹캠 모드 선택${NC}"
        ;;
    *)
        echo "테스트 취소"
        exit 0
        ;;
esac
echo ""

# 4. FPS 설정
echo "⚙️  4단계: FPS 설정"
read -p "FPS (권장: 10, 범위: 1-30): " FPS
FPS=${FPS:-10}
echo ""

# 5. 지속 시간 설정 (테스트 패턴만)
if [ "$MODE" = "test" ]; then
    echo "⏱️  5단계: 테스트 지속 시간"
    read -p "지속 시간 초 (기본: 30): " DURATION
    DURATION=${DURATION:-30}
    echo ""
fi

# 6. Vision Server 시작 확인
echo "========================================"
echo "🚀 Vision Server 시작이 필요합니다"
echo "========================================"
echo ""
echo "새 터미널을 열고 다음 명령을 실행하세요:"
echo -e "${YELLOW}  ./ai_server/start_vision_server.sh${NC}"
echo ""
read -p "Vision Server가 시작되었습니까? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Vision Server를 먼저 시작해주세요"
    exit 0
fi
echo ""

# 7. Camera Simulator 실행
echo "========================================"
echo "📹 Camera Simulator 시작"
echo "========================================"
echo ""
echo "설정:"
echo "  - 모드: $MODE"
echo "  - FPS: $FPS"
if [ "$MODE" = "test" ]; then
    echo "  - 지속 시간: ${DURATION}초"
fi
echo ""
read -p "시작하시겠습니까? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "테스트 취소"
    exit 0
fi

echo ""
echo "========================================"
echo "🎬 카메라 스트리밍 시작!"
echo "========================================"
echo ""
echo "종료하려면 Ctrl+C를 누르세요"
echo ""

# Camera Simulator 실행
if [ "$MODE" = "test" ]; then
    python3 robot/camera_simulator.py --mode test --fps $FPS --duration $DURATION
else
    python3 robot/camera_simulator.py --mode webcam --fps $FPS
fi

echo ""
echo "========================================"
echo "✅ 테스트 완료!"
echo "========================================"
