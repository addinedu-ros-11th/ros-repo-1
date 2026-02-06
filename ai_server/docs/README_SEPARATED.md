# AI Server - 분리된 아키텍처

AI Inference 서버 - **LLM과 Vision이 독립적인 서버**로 운영

## 🏗️ 아키텍처 변경

### Before (통합)
```
Main Server → [gRPC] → AI Server (Port 50051)
                          ├─ LLM Service
                          └─ Vision Service
```

### After (분리) ✅
```
Main Server → [gRPC] → LLM Server (Port 50051)
            → [gRPC] → Vision Server (Port 50052)
```

## 📦 구조

```
ai_server/
├── llm_server.py              # LLM 전용 서버 (Port 50051)
├── vision_server.py           # Vision 전용 서버 (Port 50052)
├── server.py                  # (레거시) 통합 서버
├── config.py                  # 서버 설정
├── start_llm_server.sh       # LLM 서버 시작
├── start_vision_server.sh    # Vision 서버 시작
├── start_all_servers.sh      # 모든 서버 시작
├── test_client.py            # 테스트 클라이언트
├── .env.example              # 환경 변수 예시
├── services/                 # AI 서비스 레이어
│   ├── llm_service.py       # LLM 서비스 (Qwen3-4B)
│   └── vision_service.py    # Vision 서비스 (YOLOv8n)
└── grpc_impl/               # gRPC 구현
    ├── ai_services.proto    # 분리된 서비스 정의
    ├── ai_inference.proto   # (레거시) 통합 proto
    ├── llm_servicer.py      # LLM Servicer
    └── vision_servicer.py   # Vision Servicer
```

## 🚀 실행 방법

### 방법 1: 개별 서버 시작 (권장)

```bash
# 터미널 1: LLM 서버
./ai_server/start_llm_server.sh

# 터미널 2: Vision 서버
./ai_server/start_vision_server.sh
```

### 방법 2: 모든 서버 한번에 시작

```bash
./ai_server/start_all_servers.sh
```

### 방법 3: 직접 실행

```bash
# LLM 서버
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
python3 -m ai_server.llm_server

# Vision 서버 (별도 터미널)
python3 -m ai_server.vision_server
```

## 🔌 Main Server 통신

### LLM Client 사용

```python
from main_server.core_layer.ai_inference.llm_client import LLMClient

llm = LLMClient()  # Port 50051

# 텍스트 생성
result = await llm.generate_text("안녕하세요")
print(result["generated_text"])

# 대화
messages = [
    {"role": "user", "content": "오늘 날씨 어때?"}
]
result = await llm.chat(messages)
print(result["response"])

# 의도 분석
result = await llm.analyze_intent("회의실 예약해줘")
print(result["intent"])  # "booking"
```

### Vision Client 사용

```python
from main_server.core_layer.ai_inference.vision_client import VisionClient

vision = VisionClient()  # Port 50052

# 객체 인식
result = await vision.detect_objects("image_123")
print(result["object_name"])

# 얼굴 인식
result = await vision.recognize_faces("image_456")
print(result["person_type"])

# 복수 객체 인식
results = await vision.detect_multiple_objects("image_789")
for obj in results:
    print(f"{obj['object_name']}: {obj['confidence']}")
```

## 💡 분리 아키텍처의 장점

### 1. **독립적 스케일링**
```bash
# Vision 서버만 3개 인스턴스로 확장
./start_vision_server.sh  # Port 50052
./start_vision_server.sh  # Port 50053
./start_vision_server.sh  # Port 50054

# LLM 서버는 1개만 유지
./start_llm_server.sh     # Port 50051
```

### 2. **장애 격리**
- Vision 서버 다운 → LLM 서비스는 정상 작동
- LLM 서버 다운 → Vision 서비스는 정상 작동

### 3. **하드웨어 최적화**
```
LLM Server  → CPU/메모리 집약적 서버
Vision Server → GPU 탑재 서버
```

### 4. **독립 배포**
```bash
# Vision만 업데이트
git pull
./start_vision_server.sh

# LLM은 기존 버전 유지
```

## 🔧 환경 설정

### .env 파일
```bash
# LLM Server (Port 50051)
LLM_GRPC_HOST=0.0.0.0
LLM_GRPC_PORT=50051
LLM_MODEL_NAME=qwen3-4b
LLM_MODEL_PATH=./models/qwen3-4b

# Vision Server (Port 50052)
VISION_GRPC_HOST=0.0.0.0
VISION_GRPC_PORT=50052
VISION_MODEL_NAME=yolov8n
VISION_MODEL_PATH=./models/yolov8n.pt

# Server Settings
MAX_WORKERS=10
LOG_LEVEL=INFO
```

## 📊 포트 할당

| 서비스 | 포트 | 용도 |
|--------|------|------|
| LLM Server | 50051 | 자연어 처리, 대화, 의도 분석 |
| Vision Server | 50052 | 객체/얼굴 인식, 비전 스트리밍 |
| Main Server | 8000 | 웹 API, 관리 |

## 🐛 트러블슈팅

### 포트 충돌
```bash
# 사용 중인 포트 확인
netstat -tuln | grep 50051
netstat -tuln | grep 50052

# 프로세스 종료
kill $(lsof -t -i:50051)
kill $(lsof -t -i:50052)
```

### 연결 실패
```bash
# 서버 상태 확인
ps aux | grep llm_server
ps aux | grep vision_server

# 로그 확인 (LOG_LEVEL=DEBUG)
```

## 📈 성능 비교

| 항목 | 통합 방식 | 분리 방식 |
|------|----------|----------|
| 배포 복잡도 | ⭐ 낮음 | ⭐⭐ 보통 |
| 스케일링 | ⭐⭐ 제한적 | ⭐⭐⭐ 유연 |
| 장애 격리 | ⭐ 없음 | ⭐⭐⭐ 완전 |
| 리소스 최적화 | ⭐⭐ 제한적 | ⭐⭐⭐ 최적 |
| 독립 배포 | ⭐ 불가 | ⭐⭐⭐ 가능 |
| **대규모 추천** | ❌ | ✅ |

## 🔄 마이그레이션 가이드

### 기존 코드 (통합)
```python
from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService

ai = AIInferenceService()  # 하나의 클라이언트
await ai.request_object_detection("img")
```

### 새 코드 (분리)
```python
from main_server.core_layer.ai_inference.llm_client import LLMClient
from main_server.core_layer.ai_inference.vision_client import VisionClient

llm = LLMClient()      # Port 50051
vision = VisionClient()  # Port 50052

await llm.generate_text("Hello")
await vision.detect_objects("img")
```

## 📚 참고 문서

- [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - 상세 통합 가이드
- [ai_services.proto](grpc_impl/ai_services.proto) - gRPC 서비스 정의
- Main Server LLM Client: [llm_client.py](../main_server/core_layer/ai_inference/llm_client.py)
- Main Server Vision Client: [vision_client.py](../main_server/core_layer/ai_inference/vision_client.py)

## ✨ 다음 단계

1. **Proto 재생성**: `ai_services.proto` 기반 Python 코드 생성
2. **실제 모델 통합**: Qwen3-4B 및 YOLOv8n 로딩
3. **로드 밸런싱**: 여러 인스턴스 간 요청 분산
4. **모니터링**: Prometheus + Grafana
5. **CI/CD**: 독립적인 배포 파이프라인
