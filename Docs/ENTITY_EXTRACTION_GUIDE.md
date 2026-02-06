# LLM 엔티티 추출 기능 - 사용 가이드

## 개요
웹에서 입력받은 자연어 프롬프트에서 **장소(location)**와 **물품(item)** 정보를 추출하는 기능입니다.
- 로컬 LLM 모델 (Ollama): `qwen3:4b-instruct-2507-q4_K_M`
- gRPC 통신을 통한 Main 서버와 AI 서버 간 연동
- 독립 테스트 가능

## 기능
사용자 입력에서 다음을 추출합니다:
- **장소 (location)**: 회의실, 로비, 301호, A동 2층 등
- **물품 (item)**: 커피, 서류, 노트북, 박스 등
- **의도 (intent)**: 추출된 정보에 따라 자동 분류
  - `deliver_item_to_location`: 장소 + 물품
  - `navigate_to_location`: 장소만
  - `find_item`: 물품만
  - `unknown`: 엔티티 없음

## 아키텍처
```
웹 입력 → Main Server → gRPC → LLM Server (AI Server) → Ollama LLM
                                    ↓
                              엔티티 추출 (장소/물품)
                                    ↓
                            Main Server로 결과 반환
```

## 설치 및 설정

### 1. 필수 패키지 설치
```bash
# Python 가상환경 활성화
source .venv/bin/activate

# ollama 패키지 설치 (이미 설치됨)
pip install ollama
```

### 2. Ollama 모델 확인
```bash
# 모델이 설치되어 있는지 확인
ollama list

# qwen3:4b-instruct-2507-q4_K_M 모델이 있어야 함
# 없다면 설치:
# ollama pull qwen3:4b-instruct-2507-q4_K_M
```

### 3. 환경 변수 설정
`.env` 파일에서 LLM 모델 이름 확인:
```env
LLM_MODEL_NAME=qwen3:4b-instruct-2507-q4_K_M
LLM_GRPC_PORT=50051
```

## 사용 방법

### 방법 1: 간단한 직접 테스트 (추천)
LLM 서비스만 단독으로 테스트:

```bash
# Python 환경에서 실행
python tests/test_entity_extraction_simple.py
```

**장점**: gRPC 서버 없이 빠르게 테스트 가능

**출력 예시**:
```
[테스트 1]
입력: 회의실로 커피 가져다줘
----------------------------------------
📍 장소: 회의실
📦 물품: 커피
✓ 신뢰도: 0.90
```

### 방법 2: gRPC 서버를 통한 전체 플로우 테스트
실제 통신 환경과 동일하게 테스트:

```bash
# 1. LLM 서버 시작 (별도 터미널)
python -m ai_server.llm_server

# 2. gRPC 테스트 실행 (다른 터미널)
python tests/test_entity_extraction_grpc.py
```

**장점**: Main 서버와 동일한 통신 방식 테스트

**출력 예시**:
```
[테스트 1]
입력: 회의실로 커피 가져다줘
----------------------------------------
🎯 Intent: deliver_item_to_location
✓ Confidence: 0.90
📋 Entities:
  📍 location: 회의실 (conf: 0.90)
  📦 item: 커피 (conf: 0.90)
```

## 테스트 케이스 결과

| 입력 | 장소 | 물품 | Intent |
|------|------|------|--------|
| 회의실로 커피 가져다줘 | 회의실 | 커피 | deliver_item_to_location |
| 301호에 서류 전달해줘 | 301호 | 서류 | deliver_item_to_location |
| 로비로 이동해줘 | 로비 | - | navigate_to_location |
| 노트북 찾아줘 | - | 노트북 | find_item |
| 3층 회의실에 물 한잔 가져다줘 | 3층 회의실 | 물 | deliver_item_to_location |
| 안녕하세요 | - | - | unknown |
| A동 2층으로 가서 박스를 가져와줘 | A동 2층 | 박스 | deliver_item_to_location |

## API 사용법 (Main 서버 연동)

### gRPC 클라이언트 예시
```python
import grpc
from ai_server.grpc_impl import ai_services_pb2
from ai_server.grpc_impl import ai_services_pb2_grpc

# gRPC 채널 생성
channel = grpc.insecure_channel('localhost:50051')
stub = ai_services_pb2_grpc.LLMServiceStub(channel)

# 엔티티 추출 요청
request = ai_services_pb2.TextRequest(
    text="회의실로 커피 가져다줘",
    max_length=200
)

# AnalyzeIntent RPC 호출
response = stub.AnalyzeIntent(request)

# 결과 확인
print(f"Intent: {response.intent}")
for entity in response.entities:
    print(f"  {entity.type}: {entity.value} (conf: {entity.confidence})")
```

### 응답 구조
```protobuf
message IntentResponse {
  string intent = 1;              // deliver_item_to_location, navigate_to_location, find_item, unknown
  float confidence = 2;            // 0.0 ~ 1.0
  repeated Entity entities = 3;    // 추출된 엔티티 목록
}

message Entity {
  string type = 1;      // "location" 또는 "item"
  string value = 2;     // "회의실", "커피" 등
  float confidence = 3; // 0.0 ~ 1.0
}
```

## 구현 세부사항

### LLM 프롬프트 엔지니어링
```python
system_prompt = """당신은 사무실 로봇을 위한 명령어 파서입니다.
사용자의 요청에서 다음 정보만 추출하세요:
1. location (장소): 예) 회의실, 로비, 사무실, 301호, A동 등
2. item (물품): 예) 커피, 서류, 노트북, 상자 등

반드시 다음 JSON 형식으로만 답변하세요:
{"location": "장소이름 또는 null", "item": "물품이름 또는 null"}
"""
```

- **Temperature**: 0.1 (일관된 응답)
- **출력 형식**: JSON (구조화된 파싱)
- **Fallback**: JSON 파싱 실패 시 오류 처리

### 파일 구조
```
ai_server/
├── services/
│   └── llm_service.py           # Ollama 연동 및 엔티티 추출 로직
├── grpc_impl/
│   ├── llm_servicer.py          # gRPC Servicer (AnalyzeIntent RPC)
│   ├── ai_services.proto         # gRPC 서비스 정의
│   ├── ai_services_pb2.py       # 생성된 protobuf 메시지
│   └── ai_services_pb2_grpc.py  # 생성된 gRPC stub
├── docs/                         # 문서 파일들
├── scripts/                      # 서버 시작 스크립트들
├── llm_server.py                 # LLM gRPC 서버 실행 (포트 50051)
└── config.py                     # 설정 파일

tests/
├── test_entity_extraction_simple.py   # 간단한 테스트 스크립트
├── test_entity_extraction_grpc.py     # gRPC 테스트 스크립트
└── ai_server/                          # AI 서버 관련 테스트들
```

## 트러블슈팅

### 1. Ollama 연결 실패
```bash
# Ollama 서비스 확인
systemctl status ollama

# 또는 수동 실행
ollama serve
```

### 2. 모델 로딩 실패
```bash
# 모델 다시 다운로드
ollama pull qwen3:4b-instruct-2507-q4_K_M
```

### 3. gRPC 연결 실패
```bash
# LLM 서버가 실행 중인지 확인
ps aux | grep llm_server

# 포트 확인
netstat -tuln | grep 50051
```

### 4. JSON 파싱 오류
- LLM 응답이 예상한 JSON 형식이 아닐 경우 발생
- `raw_text` 필드에서 실제 응답 확인 가능
- Temperature를 더 낮추거나 프롬프트 수정 필요

## 향후 확장

### Main 서버 연동
1. Main 서버의 웹 인터페이스에서 사용자 입력 받기
2. gRPC 클라이언트로 LLM 서버에 요청
3. 추출된 엔티티로 Task 생성
4. Fleet Manager를 통해 로봇 제어

### 추가 엔티티 타입
- 사람 이름 (person)
- 시간 정보 (time)
- 수량 (quantity)
- 우선순위 (priority)

### 대화 히스토리 지원
- 이전 대화 맥락 유지
- 대명사 해석 ("거기", "그거")

## 참고 자료
- Ollama 문서: https://ollama.ai/
- gRPC Python: https://grpc.io/docs/languages/python/
- Qwen 모델: https://huggingface.co/Qwen

## 문의
엔티티 추출 정확도를 높이려면:
1. 프롬프트 엔지니어링 조정 (system_prompt)
2. Temperature 값 조정
3. 더 큰 모델 사용 (Qwen3-14B 등)
4. Fine-tuning 고려
