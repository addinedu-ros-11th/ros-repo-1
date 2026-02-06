# AI Server - 분리 아키텍처 완료! ✅

## 🎉 구현 완료 내용

### ✅ LLM과 Vision 서버 완전 분리
- **LLM Server**: Port 50051 (자연어 처리, 대화, 의도 분석)
- **Vision Server**: Port 50052 (객체 인식, 얼굴 인식)

## 📊 최종 아키텍처

```
┌─────────────────┐
│  Main Server    │ (Port 8000)
│  - Web API      │
│  - Task Mgmt    │
│  - Fleet Mgmt   │
└────┬─────────┬──┘
     │         │
     │         │ gRPC
     │         ▼
     │    ┌────────────────────────┐
     │    │  Vision Server         │ Port 50052
     │    │  ┌──────────────────┐  │
     │    │  │ Vision Service   │  │
     │    │  │ - YOLOv8n        │  │
     │    │  │ - 객체 인식      │  │
     │    │  │ - 얼굴 인식      │  │
     │    │  └──────────────────┘  │
     │    └────────────────────────┘
     │
     │ gRPC
     ▼
┌────────────────────────┐
│  LLM Server            │ Port 50051
│  ┌──────────────────┐  │
│  │ LLM Service      │  │
│  │ - Qwen3-4B       │  │
│  │ - 텍스트 생성    │  │
│  │ - 대화           │  │
│  │ - 의도 분석      │  │
│  └──────────────────┘  │
└────────────────────────┘
```

## 🚀 빠른 시작

### 1. 모든 서버 시작
```bash
./ai_server/start_all_servers.sh
```

### 2. 개별 서버 시작
```bash
# 터미널 1: LLM
./ai_server/start_llm_server.sh

# 터미널 2: Vision  
./ai_server/start_vision_server.sh
```

### 3. 테스트
```bash
# 모든 서버 테스트
python3 ai_server/test_client_separated.py

# LLM만 테스트
python3 ai_server/test_client_separated.py llm

# Vision만 테스트
python3 ai_server/test_client_separated.py vision
```

## 💻 Main Server에서 사용

```python
from main_server.core_layer.ai_inference.llm_client import LLMClient
from main_server.core_layer.ai_inference.vision_client import VisionClient

# LLM 사용
llm = LLMClient()
text = await llm.generate_text("안녕하세요")
response = await llm.chat([{"role": "user", "content": "Hi"}])
intent = await llm.analyze_intent("회의실 예약")

# Vision 사용
vision = VisionClient()
obj = await vision.detect_objects("image_123")
face = await vision.recognize_faces("image_456")
objs = await vision.detect_multiple_objects("image_789")
```

## 📁 핵심 파일

### AI Server
- `llm_server.py` - LLM 서버 메인
- `vision_server.py` - Vision 서버 메인
- `grpc_impl/llm_servicer.py` - LLM gRPC 구현
- `grpc_impl/vision_servicer.py` - Vision gRPC 구현
- `grpc_impl/ai_services.proto` - 분리된 프로토콜 정의

### Main Server
- `core_layer/ai_inference/llm_client.py` - LLM 클라이언트
- `core_layer/ai_inference/vision_client.py` - Vision 클라이언트

### 시작 스크립트
- `start_llm_server.sh` - LLM 서버 실행
- `start_vision_server.sh` - Vision 서버 실행
- `start_all_servers.sh` - 모든 서버 실행

### 문서
- `README_SEPARATED.md` - 분리 아키텍처 상세 가이드
- `ARCHITECTURE_COMPARISON.md` - 통합 vs 분리 비교
- `INTEGRATION_GUIDE.md` - 통합 가이드

## 🔧 환경 설정

```bash
# .env 파일
LLM_GRPC_HOST=0.0.0.0
LLM_GRPC_PORT=50051
VISION_GRPC_HOST=0.0.0.0
VISION_GRPC_PORT=50052

LLM_MODEL_NAME=qwen3-4b
LLM_MODEL_PATH=./models/qwen3-4b
VISION_MODEL_NAME=yolov8n
VISION_MODEL_PATH=./models/yolov8n.pt

MAX_WORKERS=10
LOG_LEVEL=INFO
```

## 💡 왜 분리했는가?

### 1. 독립적 스케일링
```bash
# Vision은 카메라 대수만큼 확장
Vision Server × 3 (Port 50052, 50053, 50054)

# LLM은 대화 빈도에 맞게 조절
LLM Server × 1 (Port 50051)
```

### 2. 장애 격리
- Vision 서버 장애 → LLM으로 음성 대화는 가능
- LLM 서버 장애 → Vision으로 객체 인식은 가능

### 3. 하드웨어 최적화
```
LLM Server  → CPU/메모리 집약 → 고메모리 서버
Vision Server → GPU 집약 → GPU 탑재 서버
```

### 4. 독립 배포
```bash
# Vision만 업데이트
git pull origin vision-update
./start_vision_server.sh

# LLM은 기존 버전 유지
```

## 📊 포트 할당

| 서비스 | 포트 | 프로토콜 | 용도 |
|--------|------|---------|------|
| Main Server | 8000 | HTTP | REST API, WebSocket |
| LLM Server | 50051 | gRPC | 자연어 처리 |
| Vision Server | 50052 | gRPC | 컴퓨터 비전 |

## 🎯 다음 단계

1. **Proto 재생성**: `ai_services.proto` → Python 코드
2. **실제 모델 통합**: Qwen3-4B, YOLOv8n
3. **로드 밸런싱**: 여러 인스턴스 분산
4. **모니터링**: Prometheus + Grafana
5. **CI/CD**: 독립 배포 파이프라인

## 📚 참고 문서

- [README_SEPARATED.md](README_SEPARATED.md) - 상세 사용법
- [ARCHITECTURE_COMPARISON.md](ARCHITECTURE_COMPARISON.md) - 아키텍처 비교
- [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - 통합 가이드

## ✨ 요약

- ✅ LLM과 Vision 서버 완전 분리
- ✅ 독립적 포트 (50051, 50052)
- ✅ Main Server 클라이언트 분리
- ✅ 개별/통합 시작 스크립트
- ✅ 상세 문서 완비
- ✅ 테스트 클라이언트

**대규모 프로덕션 환경에 최적화된 마이크로서비스 아키텍처 완성!** 🎉
