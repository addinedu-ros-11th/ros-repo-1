# AI Server Integration Guide

## 📋 개요

AI Server는 Main Server와 gRPC 통신을 통해 LLM(Qwen3-4B)과 Vision(YOLOv8n) 기능을 제공합니다.

## 🏗️ 아키텍처

```
┌─────────────────┐         gRPC          ┌─────────────────┐
│                 │◄─────────────────────►│                 │
│   Main Server   │   (Port: 50051)       │   AI Server     │
│                 │                       │                 │
│  - Web API      │                       │  - LLM Service  │
│  - Task Mgmt    │                       │  - Vision Svc   │
│  - Fleet Mgmt   │                       │                 │
└─────────────────┘                       └─────────────────┘
```

## 📁 AI Server 구조

```
ai_server/
├── server.py                    # gRPC 서버 메인
├── config.py                    # 설정
├── test_client.py              # 테스트 클라이언트
├── start_ai_server.sh          # 시작 스크립트
├── .env.example                # 환경 변수 예시
├── README.md                   # AI Server 문서
├── services/                   # AI 서비스 레이어
│   ├── llm_service.py         # LLM (Qwen3-4B)
│   └── vision_service.py      # Vision (YOLOv8n)
└── grpc_impl/                 # gRPC 구현
    ├── ai_inference.proto
    ├── ai_inference_pb2.py
    ├── ai_inference_pb2_grpc.py
    └── ai_inference_servicer.py
```

## 🚀 실행 방법

### 1. 환경 설정

```bash
# .env 파일 생성
cp ai_server/.env.example .env

# 또는 직접 .env 파일에 추가
cat >> .env << EOF

# AI Server Configuration
AI_INFERENCE_GRPC_HOST=0.0.0.0
AI_INFERENCE_GRPC_PORT=50051
LLM_MODEL_NAME=qwen3-4b
LLM_MODEL_PATH=./models/qwen3-4b
VISION_MODEL_NAME=yolov8n
VISION_MODEL_PATH=./models/yolov8n.pt
MAX_WORKERS=10
LOG_LEVEL=INFO
EOF
```

### 2. AI Server 시작

**방법 1: 스크립트 사용**
```bash
./ai_server/start_ai_server.sh
```

**방법 2: 직접 실행**
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
python3 -m ai_server.server
```

### 3. Main Server 시작

```bash
# 별도 터미널에서
python3 -m uvicorn main_server.app:app --host 0.0.0.0 --port 8000
```

### 4. 연결 테스트

```bash
# AI Server 테스트 클라이언트 실행
python3 ai_server/test_client.py
```

## 🔌 통신 API

### DetectObjects (객체 인식)

**Main Server에서 사용:**
```python
from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService

ai_service = AIInferenceService()
result = await ai_service.request_object_detection("image_123")

# 결과:
# {
#     "object_name": "person",
#     "confidence": 0.95,
#     "box": {"x": 100, "y": 150, "width": 200, "height": 300}
# }
```

### RecognizeFaces (얼굴 인식)

**Main Server에서 사용:**
```python
result = await ai_service.request_face_recognition("image_456")

# 결과:
# {
#     "person_type": "Employee",
#     "employee_id": "EMP001",
#     "confidence": 0.92
# }
```

### StreamInferenceResults (실시간 스트리밍)

**Main Server에서 사용:**
```python
# main_server/core_layer/ai_inference/grpc_inference_client.py 에서
# 스트리밍 메서드 구현 필요

async for result in ai_service.stream_inference_results():
    print(f"Robot: {result['robot_id']}, Result: {result}")
```

## 🔧 개발 상태

### ✅ 완료된 기능
- gRPC 서버 구조
- Main Server와 통신 인터페이스
- LLM Service 레이어 (스텁)
- Vision Service 레이어 (스텁)
- 기본 에러 핸들링
- 로깅 시스템

### 🚧 향후 작업
- [ ] Qwen3-4B 모델 통합
- [ ] YOLOv8n 모델 통합
- [ ] 이미지 데이터 전송 메커니즘
- [ ] 모델 캐싱 및 최적화
- [ ] 배치 처리 지원
- [ ] 성능 모니터링
- [ ] 단위 테스트

## 📝 개발 가이드

### 실제 LLM 모델 통합 시

[ai_server/services/llm_service.py](ai_server/services/llm_service.py) 수정:

```python
def initialize(self):
    from transformers import AutoModelForCausalLM, AutoTokenizer
    
    self.tokenizer = AutoTokenizer.from_pretrained(self.model_path)
    self.model = AutoModelForCausalLM.from_pretrained(
        self.model_path,
        device_map="auto",
        torch_dtype=torch.float16
    )
    logger.info("Qwen3-4B 모델 로딩 완료")
```

### 실제 Vision 모델 통합 시

[ai_server/services/vision_service.py](ai_server/services/vision_service.py) 수정:

```python
def initialize(self):
    from ultralytics import YOLO
    
    self.model = YOLO(self.model_path)
    logger.info("YOLOv8n 모델 로딩 완료")

def detect_objects(self, image_id: str):
    # 이미지 로드
    image = load_image(image_id)
    
    # YOLO 추론
    results = self.model(image)
    
    # 결과 파싱
    for result in results:
        boxes = result.boxes
        # ... 처리
```

## 🐛 트러블슈팅

### 1. gRPC 연결 실패
```bash
# AI Server 상태 확인
ps aux | grep "ai_server.server"

# 포트 확인
netstat -tuln | grep 50051
```

### 2. Import 오류
```bash
# PYTHONPATH 설정 확인
echo $PYTHONPATH

# 프로젝트 루트가 포함되어야 함
export PYTHONPATH="${PYTHONPATH}:/home/dh/dev_ws/git_ws/ros-repo-1"
```

### 3. 로그 확인
AI Server는 기본적으로 INFO 레벨로 로그를 출력합니다:
```bash
# 더 자세한 로그를 원하면 .env 수정
LOG_LEVEL=DEBUG
```

## 📚 참고 자료

- [AI Server README](ai_server/README.md)
- [gRPC Python Documentation](https://grpc.io/docs/languages/python/)
- [Protocol Buffers](https://developers.google.com/protocol-buffers)
- Main Server gRPC Client: [main_server/core_layer/ai_inference/grpc_inference_client.py](main_server/core_layer/ai_inference/grpc_inference_client.py)

## 📞 연락처

문제가 발생하면 이슈를 등록하거나 개발팀에 문의하세요.
