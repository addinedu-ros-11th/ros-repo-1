# AI Server

AI Inference 서버 - LLM 및 Vision 기능을 제공하는 gRPC 서버

## 구조

```
ai_server/
├── server.py                # gRPC 서버 메인 엔트리 포인트
├── config.py               # 서버 설정
├── services/               # AI 서비스 레이어
│   ├── llm_service.py     # LLM 서비스 (Qwen3-4B)
│   └── vision_service.py  # Vision 서비스 (YOLOv8n)
└── grpc_impl/             # gRPC 구현
    ├── ai_inference.proto
    ├── ai_inference_pb2.py
    ├── ai_inference_pb2_grpc.py
    └── ai_inference_servicer.py
```

## 기능

### 1. LLM 파트 (Qwen3-4B)
- 자연어 처리
- 대화형 응답 생성
- 의도 분석
- *현재는 스텁 구현, 실제 모델 통합은 추후 진행*

### 2. Vision 파트 (YOLOv8n)
- 객체 인식 (Object Detection)
- 얼굴 인식 (Face Recognition)
- *현재는 스텁 구현, 실제 모델 통합은 추후 진행*

## gRPC 서비스

### DetectObjects
이미지에서 객체를 인식합니다.

**Request:**
```protobuf
message ImageRequest {
  string image_id = 1;
}
```

**Response:**
```protobuf
message ObjectDetectionResponse {
  string object_name = 1;
  float confidence = 2;
  BoundingBox box = 3;
}
```

### RecognizeFaces
이미지에서 얼굴을 인식합니다.

**Request:**
```protobuf
message ImageRequest {
  string image_id = 1;
}
```

**Response:**
```protobuf
message FaceRecognitionResponse {
  string person_type = 1;
  optional string employee_id = 2;
  float confidence = 3;
}
```

### StreamInferenceResults
실시간 추론 결과를 스트리밍합니다.

**Request:**
```protobuf
message Empty {}
```

**Response Stream:**
```protobuf
message InferenceResult {
  string robot_id = 1;
  oneof result {
    ObjectDetectionResponse object_detection = 2;
    FaceRecognitionResponse face_recognition = 3;
  }
}
```

## 실행 방법

### 1. 환경 변수 설정 (.env)
```bash
# AI Inference Server
AI_INFERENCE_GRPC_HOST=0.0.0.0
AI_INFERENCE_GRPC_PORT=50051

# LLM Configuration
LLM_MODEL_NAME=qwen3-4b
LLM_MODEL_PATH=./models/qwen3-4b

# Vision Configuration
VISION_MODEL_NAME=yolov8n
VISION_MODEL_PATH=./models/yolov8n.pt

# Server Settings
MAX_WORKERS=10
LOG_LEVEL=INFO
```

### 2. 서버 실행
```bash
# 프로젝트 루트에서 실행
python -m ai_server.server

# 또는
cd ai_server
python server.py
```

### 3. Main 서버와 통신 확인
Main 서버가 실행 중이면 자동으로 AI 서버와 연결됩니다.

```python
# Main 서버에서 사용 예시
from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService

ai_service = AIInferenceService()
result = await ai_service.request_object_detection("image_001")
print(result)
```

## 개발 상태

### 완료
- ✅ gRPC 서버 구조 구축
- ✅ Main 서버와 데이터 통신 구현
- ✅ LLM 서비스 레이어 (스텁)
- ✅ Vision 서비스 레이어 (스텁)
- ✅ gRPC Servicer 구현

### 향후 작업
- ⬜ Qwen3-4B 모델 통합
- ⬜ YOLOv8n 모델 통합
- ⬜ 이미지 데이터 처리 파이프라인
- ⬜ 모델 최적화 및 성능 튜닝
- ⬜ 에러 핸들링 강화
- ⬜ 로깅 및 모니터링
- ⬜ 단위 테스트 작성

## 참고사항

- 현재는 실제 AI 모델 없이 통신 구조만 구현되어 있습니다.
- 모든 추론 결과는 더미 데이터를 반환합니다.
- 실제 모델 통합 시 `services/` 디렉토리의 TODO 부분을 구현하면 됩니다.
