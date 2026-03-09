#!/bin/bash

# ==============================================================================
# Pinky Pro 로봇 자동 실행(Autostart) 설정 스크립트
#
# 사용법:
# 1. 이 파일을 로봇의 홈 디렉토리 또는 작업 공간으로 복사하세요.
# 2. 실행 권한 부여: chmod +x setup_autostart.sh
# 3. 스크립트 실행: ./setup_autostart.sh
# 4. 로그 확인: journalctl -u pinky_robot.service -f
#
# 주의사항:
# - 상단의 ROBOT_NS와 ROBOT_ID를 로봇에 맞게 수정하세요.
# - 이미 서비스가 돌고 있다면 자동으로 재시작됩니다.
# ==============================================================================

# --- [설정] 로봇별 고유값 (여기만 수정하세요) ---
ROBOT_NS="robot_2"
ROBOT_ID="2"
# ------------------------------------------------

# 현재 사용자 및 경로 자동 감지
USER_NAME=$(whoami)
USER_HOME=$(eval echo ~$USER_NAME)
WORKSPACE_DIR="$USER_HOME/pinky_pro"
ROS_DISTRO="jazzy"

echo "=========================================="
echo " Pinky Pro 자동 실행 서비스 설치"
echo " 사용자: $USER_NAME"
echo " 작업공간: $WORKSPACE_DIR"
echo " 타겟 로봇: $ROBOT_NS (ID: $ROBOT_ID)"
echo " ROS 버전: $ROS_DISTRO"
echo "=========================================="

# 서비스 파일 경로
SERVICE_FILE="/etc/systemd/system/pinky_robot.service"

# 서비스 파일 내용 생성
# - After=network.target: 네트워크 연결 후 실행
# - Environment: ROS 2 통신을 위한 필수 환경 변수 설정
# - ExecStart: ROS 2 Launch 실행 (source 명령어 포함)
SERVICE_CONTENT="[Unit]
Description=Pinky Pro Robot Autostart Service
After=network.target network-online.target time-sync.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$WORKSPACE_DIR
# DDS 통신 설정 (멀티 로봇 환경 필수)
Environment=\"ROS_DOMAIN_ID=0\"
Environment=\"ROS_LOCALHOST_ONLY=0\"
Environment=\"RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\" 
# 실행 명령어 (환경 설정 로드 -> 런칭)
ExecStart=/bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $WORKSPACE_DIR/install/setup.bash && ros2 launch pinky_app pinky_app.launch.py robot_ns:=$ROBOT_NS robot_id:=$ROBOT_ID'
# 프로세스 종료 시 자동 재시작 정책
Restart=always
RestartSec=10s
StartLimitInterval=60s
StartLimitBurst=5

[Install]
WantedBy=multi-user.target"

# 1. 기존 서비스 중지 (있다면)
if systemctl is-active --quiet pinky_robot.service; then
    echo "기존 서비스 중지 중..."
    sudo systemctl stop pinky_robot.service
fi

# 2. 서비스 파일 생성 (sudo 권한 필요)
echo "1. 서비스 파일 생성 중: $SERVICE_FILE"
echo "$SERVICE_CONTENT" | sudo tee $SERVICE_FILE > /dev/null

# 3. Systemd 리로드 및 활성화
echo "2. Systemd 데몬 리로드"
sudo systemctl daemon-reload

echo "3. 서비스 활성화 (부팅 시 자동 실행)"
sudo systemctl enable pinky_robot.service

echo "4. 서비스 즉시 시작"
sudo systemctl start pinky_robot.service

echo "=========================================="
echo "✅ 설치 완료!"
echo "상태 확인: sudo systemctl status pinky_robot.service"
echo "로그 보기: journalctl -u pinky_robot.service -f"
echo "=========================================="
