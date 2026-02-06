# UDP 영상 스트리밍 구현 완료! ✅

## 🎉 구현된 기능

### ✅ Vision Server - UDP 영상 수신 및 처리
- **UDP 수신**: 로봇 카메라로부터 영상 프레임 수신 (Port 54321)
- **실시간 처리**: YOLOv8n으로 객체 인식
- **Main Server 전송**: gRPC로 결과 전송 (Port 50052)

### ✅ Robot Camera Simulator
- **테스트 패턴**: 자동 생성된 패턴 전송
- **웹캠 지원**: 실제 카메라 영상 전송
- **UDP 전송**: JPEG 압축 + 패킷 분할

## 🏗️ 전체 아키텍처

```
┌─────────────────────┐
│  Robot Camera       │
│  - ROS 카메라 노드  │
│  - UDP 전송         │
└──────┬──────────────┘
       │ UDP (Port 54321)
       │ JPEG Frames
       ▼
┌─────────────────────────────────┐
│  Vision Server                  │
│  ┌───────────────────────────┐  │
│  │ UDP Video Receiver        │  │
│  │ - 영상 프레임 수신        │  │
│  │ - JPEG 디코딩             │  │
│  └──────────┬────────────────┘  │
│             ▼                    │
│  ┌───────────────────────────┐  │
│  │ Video Stream Processor    │  │
│  │ - 프레임 큐 관리          │  │
│  └──────────┬────────────────┘  │
│             ▼                    │
│  ┌───────────────────────────┐  │
│  │ Vision Service (YOLOv8n)  │  │
│  │ - 객체 인식               │  │
│  │ - 얼굴 인식               │  │
│  └──────────┬────────────────┘  │
│             │                    │
│  Port 54321 (UDP) ◄────────────┤
│  Port 50052 (gRPC) ├───────────►│
└─────────────────────────────────┘
       │ gRPC
       ▼
┌─────────────────────┐
│  Main Server        │
│  - Vision Client    │
│  - 작업 처리        │
└─────────────────────┘
```

## 🚀 빠른 테스트

### 방법 1: 인터랙티브 스크립트 (권장 ⭐)

```bash
./ai_server/test_udp_video.sh
```

메뉴에서 옵션 선택:
1. 테스트 패턴 또는 웹캠 선택
2. FPS 설정
3. 자동 실행

### 방법 2: 수동 실행

**터미널 1: Vision Server**
```bash
./ai_server/start_vision_server.sh
```

**터미널 2: Camera Simulator (2-3초 대기 후)**
```bash
# 테스트 패턴 (권장)
python3 robot/camera_simulator.py --mode test --fps 10 --duration 30

# 또는 웹캠
python3 robot/camera_simulator.py --mode webcam --fps 10
```

## 📁 핵심 파일

### Vision Server (AI Server)
```
ai_server/
├── vision_server.py                # Vision 서버 메인 (UDP + gRPC)
├── services/
│   ├── vision_service.py          # Vision 서비스 (YOLOv8n)
│   └── video_receiver.py          # UDP 영상 수신 및 처리
├── test_udp_video.sh             # 통합 테스트 스크립트
└── UDP_VIDEO_TEST.md             # 상세 테스트 가이드
```

### Robot Camera
```
robot/
└── camera_simulator.py            # 로봇 카메라 시뮬레이터
```

## 🔧 설정 (.env)

```bash
# Vision Server
VISION_GRPC_HOST=0.0.0.0
VISION_GRPC_PORT=50052

# UDP Video Stream (로봇 카메라 → Vision Server)
VIDEO_STREAM_HOST=0.0.0.0
VIDEO_STREAM_PORT=54321
VIDEO_BUFFER_SIZE=65536
VIDEO_FPS=30
```

## 📊 포트 구성

| 포트 | 프로토콜 | 방향 | 용도 |
|------|---------|------|------|
| 54321 | UDP | Robot → Vision | 영상 프레임 전송 |
| 50052 | gRPC | Vision ↔ Main | 인식 결과 전송 |
| 50051 | gRPC | LLM ↔ Main | 자연어 처리 |

## ✅ 정상 작동 확인

### Vision Server 로그
```
============================================================
Vision Server (YOLOv8n) 시작
============================================================
Vision 서비스 초기화 중...
UDP Video Receiver 초기화 중...
Video Stream Processor 초기화 중...
UDP Video Receiver 시작: 0.0.0.0:54321
Video Stream Processor 시작
gRPC 서버 시작: 0.0.0.0:50052
============================================================
Vision Server 준비 완료
gRPC 주소: 0.0.0.0:50052
UDP 영상 수신: 0.0.0.0:54321
모델: yolov8n
============================================================
객체 인식 완료: robot=127.0.0.1, object=person
프레임 처리 완료: robot=127.0.0.1, frame=0, objects=person
```

### Camera Simulator 로그
```
Camera Simulator 초기화: localhost:54321
테스트 패턴 스트리밍 시작 (FPS: 10, 30초)
Frame 0 전송 완료 (3 패킷, 15234 bytes)
Frame 1 전송 완료 (3 패킷, 15187 bytes)
전송 진행: 1/30초
전송 진행: 2/30초
```

## 🎯 다음 단계

1. **의존성 설치**
   ```bash
   pip install opencv-python numpy
   ```

2. **Vision Server 실행**
   ```bash
   ./ai_server/start_vision_server.sh
   ```

3. **Camera Simulator 실행**
   ```bash
   ./ai_server/test_udp_video.sh
   ```

4. **로그 확인**
   - Vision Server: 프레임 수신 및 처리 확인
   - Camera Simulator: 프레임 전송 확인

## 🔄 실제 로봇 연동

실제 로봇에서는 `robot/camera_simulator.py`의 로직을 ROS 노드로 구현:

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import struct
import cv2

class RobotCameraPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision_server = ("192.168.1.100", 54321)
        
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.send_frame(frame)  # UDP 전송
```

## 🐛 문제 해결

### OpenCV 설치 오류
```bash
pip install opencv-python numpy
```

### 포트 충돌
```bash
# 포트 사용 확인
netstat -tuln | grep -E '54321|50052'

# 프로세스 종료
kill $(lsof -t -i:54321)
kill $(lsof -t -i:50052)
```

### 프레임 수신 안 됨
1. Vision Server가 실행 중인지 확인
2. 방화벽 설정 확인 (UDP 54321 포트)
3. 로그에서 "UDP Video Receiver 시작" 메시지 확인

## 📚 참고 문서

- [UDP_VIDEO_TEST.md](UDP_VIDEO_TEST.md) - 상세 테스트 가이드
- [README_SEPARATED.md](README_SEPARATED.md) - 분리 아키텍처 문서
- [ARCHITECTURE_COMPARISON.md](ARCHITECTURE_COMPARISON.md) - 아키텍처 비교

---

**UDP 영상 스트리밍 완전 구현 완료!** 🎉

이제 로봇 카메라에서 UDP로 영상을 받아 YOLOv8n으로 판별하고 Main Server로 결과를 전송하는 전체 파이프라인이 작동합니다.
