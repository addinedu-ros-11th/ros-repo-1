# UDP 영상 스트리밍 테스트 가이드

## 🎥 전체 흐름

```
┌─────────────────┐       UDP Video        ┌──────────────────┐       gRPC        ┌────────────────┐
│  Robot Camera   │ ──────────────────────► │  Vision Server   │ ────────────────► │  Main Server   │
│  (Port 54321로  │    (JPEG frames)        │  - UDP 수신      │   (detection     │  - 결과 수신   │
│   영상 전송)    │                         │  - YOLOv8n 판별  │    results)      │  - 작업 처리   │
└─────────────────┘                         └──────────────────┘                   └────────────────┘
```

## 📋 단계별 테스트

### 1단계: 의존성 설치

```bash
cd /home/dh/dev_ws/git_ws/ros-repo-1
pip install opencv-python numpy
```

### 2단계: Vision Server 시작

**터미널 1:**
```bash
./ai_server/start_vision_server.sh
```

**확인 사항:**
- ✅ `gRPC 서버 시작: 0.0.0.0:50052`
- ✅ `UDP 영상 수신: 0.0.0.0:54321`
- ✅ `Vision Server 준비 완료`

### 3단계: Robot Camera Simulator 실행

**터미널 2 (테스트 패턴 전송):**
```bash
python3 robot/camera_simulator.py --mode test --fps 10 --duration 30
```

**또는 웹캠 사용 (웹캠이 있는 경우):**
```bash
python3 robot/camera_simulator.py --mode webcam --fps 10 --camera 0
```

**파라미터:**
- `--host`: Vision 서버 주소 (기본: localhost)
- `--port`: Vision 서버 포트 (기본: 54321)
- `--fps`: 초당 프레임 수 (기본: 30)
- `--mode`: test (테스트 패턴) 또는 webcam (실제 카메라)
- `--duration`: 테스트 지속 시간 초 (기본: 60)

### 4단계: 로그 확인

**Vision Server 터미널에서 확인:**
```
UDP Video Receiver 시작: 0.0.0.0:54321
Video Stream Processor 시작
객체 인식 완료: robot=127.0.0.1, object=person
프레임 처리 완료: robot=127.0.0.1, frame=0, objects=person
```

---

## 🚀 빠른 테스트 (올인원)

### 옵션 A: 테스트 패턴 사용

```bash
# 터미널 1: Vision Server
./ai_server/start_vision_server.sh

# 터미널 2: Camera Simulator (2-3초 대기 후)
python3 robot/camera_simulator.py --mode test --fps 10 --duration 30
```

### 옵션 B: 웹캠 사용 (웹캠이 있는 경우)

```bash
# 터미널 1: Vision Server
./ai_server/start_vision_server.sh

# 터미널 2: Webcam Streaming (2-3초 대기 후)
python3 robot/camera_simulator.py --mode webcam --fps 10
```

---

## 🔧 상세 설정

### Vision Server 설정 (.env)

```bash
# Vision Server
VISION_GRPC_HOST=0.0.0.0
VISION_GRPC_PORT=50052

# UDP Video Stream
VIDEO_STREAM_HOST=0.0.0.0
VIDEO_STREAM_PORT=54321
VIDEO_BUFFER_SIZE=65536
VIDEO_FPS=30
```

### 포트 설명

| 포트 | 프로토콜 | 용도 |
|------|---------|------|
| 50052 | gRPC | Main Server ↔ Vision Server 통신 |
| 54321 | UDP | Robot Camera → Vision Server 영상 전송 |

---

## 📊 성능 조절

### FPS 조절
```bash
# 낮은 FPS (네트워크 부하 감소)
python3 robot/camera_simulator.py --fps 5

# 중간 FPS (기본)
python3 robot/camera_simulator.py --fps 10

# 높은 FPS (실시간)
python3 robot/camera_simulator.py --fps 30
```

### 해상도 조절

`robot/camera_simulator.py` 파일에서 수정:
```python
# 프레임 크기 조절
frame = cv2.resize(frame, (640, 480))  # 주석 해제
```

---

## 🐛 문제 해결

### 1. "ModuleNotFoundError: No module named 'cv2'"
```bash
pip install opencv-python numpy
```

### 2. "Address already in use" (포트 충돌)
```bash
# 54321 포트 사용 확인
netstat -tuln | grep 54321

# 프로세스 종료
kill $(lsof -t -i:54321)
```

### 3. "프레임 수신 안 됨"

**Vision Server 로그 확인:**
```bash
# UDP Receiver가 시작되었는지 확인
grep "UDP Video Receiver 시작" vision_server.log
```

**Camera Simulator 로그 확인:**
```bash
# 프레임 전송 확인
# "Frame X 전송 완료" 메시지가 보여야 함
```

### 4. 네트워크 방화벽
```bash
# UDP 54321 포트 허용 (필요시)
sudo ufw allow 54321/udp
```

---

## 📈 실제 로봇 연동

### 로봇 측 구현 (ROS 노드 예시)

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
        self.vision_server = ("192.168.1.100", 54321)  # Vision Server 주소
        self.frame_id = 0
        
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
    
    def image_callback(self, msg):
        # ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # JPEG 인코딩 및 UDP 전송
        self.send_frame(frame)
    
    def send_frame(self, frame):
        # (camera_simulator.py의 send_frame 로직 동일)
        pass
```

---

## ✅ 정상 작동 확인

다음이 모두 보이면 성공:

**Vision Server 터미널:**
```
✅ Vision Server 준비 완료
✅ UDP 영상 수신: 0.0.0.0:54321
✅ UDP Video Receiver 시작
✅ Video Stream Processor 시작
✅ 객체 인식 완료: robot=127.0.0.1, object=person
```

**Camera Simulator 터미널:**
```
✅ Camera Simulator 초기화: localhost:54321
✅ 테스트 패턴 스트리밍 시작 (FPS: 10, 30초)
✅ Frame 0 전송 완료 (3 패킷, 15234 bytes)
✅ 전송 진행: 1/30초
✅ 전송 진행: 2/30초
```

---

## 🎯 다음 단계

1. **실제 YOLOv8n 모델 통합**: 현재는 스텁, 실제 객체 인식 구현
2. **얼굴 인식 추가**: 직원/손님 구분
3. **Main Server 연동**: gRPC로 결과 전송
4. **성능 최적화**: 프레임 건너뛰기, 배치 처리
5. **여러 로봇 지원**: robot_id로 구분

---

## 📚 관련 파일

- Vision Server: `ai_server/vision_server.py`
- UDP 수신기: `ai_server/services/video_receiver.py`
- Vision 서비스: `ai_server/services/vision_service.py`
- Camera Simulator: `robot/camera_simulator.py`
- 설정: `.env`
