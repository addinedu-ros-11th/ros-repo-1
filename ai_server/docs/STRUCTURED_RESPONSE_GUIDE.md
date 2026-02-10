# 구조화된 자연어 프롬프트 해석 기능

## 개요

Main 서버로부터 자연어 프롬프트를 받아 구조화된 작업 메시지로 변환하는 기능입니다.

## 기능

### 지원하는 작업 유형 (TaskType)

#### 1. 물품/간식 배달 관련
- `SNACK_DELIVERY`: 간식 배달
- `ITEM_DELIVERY`: 물품 배달
- `PICKUP_ITEM`: 물품 수거

#### 2. 안내/이동 관련
- `GUIDE_GUEST`: 방문객 안내
- `NAVIGATE_TO_LOCATION`: 특정 위치로 이동
- `FOLLOW_PERSON`: 사람 따라가기

#### 3. 로봇 제어 관련
- `CALL_ROBOT`: 로봇 호출
- `RETURN_TO_BASE`: 로봇 복귀 (충전소/로비)
- `CANCEL_TASK`: 작업 취소
- `PAUSE_TASK`: 작업 일시정지
- `RESUME_TASK`: 작업 재개

#### 4. 환경 제어 관련 (IoT)
- `CONTROL_LIGHT`: 조명 제어
- `CONTROL_TEMPERATURE`: 온도 제어
- `CONTROL_AC`: 에어컨 제어
- `CONTROL_DOOR`: 문 제어

#### 5. 정보 조회 관련
- `QUERY_ROBOT_STATUS`: 로봇 상태 조회
- `QUERY_LOCATION`: 위치 정보 조회
- `QUERY_AVAILABILITY`: 회의실/좌석 가능 여부 조회
- `FIND_PERSON`: 사람 찾기
- `FIND_ITEM`: 물건 찾기

#### 6. 회의실 관련
- `RESERVE_MEETING_ROOM`: 회의실 예약
- `CANCEL_RESERVATION`: 예약 취소
- `CHECK_ROOM_STATUS`: 회의실 상태 확인

#### 7. 순찰/모니터링
- `PATROL_AREA`: 구역 순찰
- `MONITOR_ENVIRONMENT`: 환경 모니터링

#### 8. 기타
- `GENERAL_QUESTION`: 일반 질문
- `GREETING`: 인사
- `UNKNOWN`: 알 수 없음

### 구조화된 메시지 필드 (StructuredMessage)

각 작업 유형에 따라 다음 필드들이 추출됩니다:

#### 공통 필드
- `location`: 목적지/장소
- `item`: 물품/간식 이름
- `person_name`: 사람 이름
- `person_id`: 사람 ID

#### 배달 관련
- `source_location`: 출발지
- `dest_location`: 목적지
- `quantity`: 수량

#### IoT 제어 관련
- `device_type`: 장치 타입 (LIGHT, THERMOSTAT, AIR_CONDITIONER, DOOR_LOCK)
- `command`: 명령 (TURN_ON, TURN_OFF, SET_VALUE, LOCK, UNLOCK)
- `target_value`: 목표 값
- `room_id`: 방 ID

#### 회의실 관련
- `meeting_room_id`: 회의실 ID
- `start_time`: 시작 시간
- `end_time`: 종료 시간
- `attendee_count`: 참석자 수

#### 순찰 관련
- `area`: 구역
- `waypoints`: 경유지 목록

#### 기타
- `query_type`: 조회 유형
- `message`: 일반 메시지
- `keywords`: 키워드 목록

## 사용 방법

### 1. LLM 서버 시작

```bash
python -m ai_server.llm_server
```

### 2. 테스트 실행

#### 방법 A: 직접 테스트 (gRPC 없이)

```bash
python tests/test_structured_response.py
```

#### 방법 B: gRPC 통합 테스트

```bash
# 서버가 실행 중인 상태에서
python tests/test_structured_response_grpc.py
```

## gRPC API 사용법

### Python 클라이언트 예시

```python
import grpc
from ai_server.grpc_impl import ai_services_pb2
from ai_server.grpc_impl import ai_services_pb2_grpc

# gRPC 채널 생성
channel = grpc.insecure_channel('localhost:50051')
stub = ai_services_pb2_grpc.LLMServiceStub(channel)

# 요청 생성
request = ai_services_pb2.NLRequest(
    req_id="req_001",
    message="회의실로 커피 갖다줘"
)

# RPC 호출
response = stub.ParseNaturalLanguage(request)

# 응답 처리
print(f"요청 ID: {response.req_id}")
print(f"작업 유형: {ai_services_pb2.TaskType.Name(response.task_type)}")
print(f"신뢰도: {response.confidence}")

# 구조화된 메시지 필드 접근
if response.struct_msg.HasField("location"):
    print(f"장소: {response.struct_msg.location}")
if response.struct_msg.HasField("item"):
    print(f"물품: {response.struct_msg.item}")
```

### Main 서버 통합 예시

```python
# main_server/core_layer/ai_inference/llm_client.py

async def parse_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
    """
    자연어 프롬프트를 구조화된 작업 메시지로 변환
    
    Args:
        req_id: 요청 ID
        message: 자연어 프롬프트
        
    Returns:
        구조화된 응답 딕셔너리
    """
    from main_server.infrastructure.grpc import ai_services_pb2
    from main_server.infrastructure.grpc import ai_services_pb2_grpc
    
    # gRPC 요청
    request = ai_services_pb2.NLRequest(
        req_id=req_id,
        message=message
    )
    
    # RPC 호출
    response = await self.stub.ParseNaturalLanguage(request)
    
    # 응답을 딕셔너리로 변환
    result = {
        "req_id": response.req_id,
        "task_type": ai_services_pb2.TaskType.Name(response.task_type),
        "confidence": response.confidence,
        "fields": {}
    }
    
    # 필드 추출
    struct_msg = response.struct_msg
    if struct_msg.HasField("location"):
        result["fields"]["location"] = struct_msg.location
    if struct_msg.HasField("item"):
        result["fields"]["item"] = struct_msg.item
    # ... 다른 필드들도 동일하게 추출
    
    return result
```

## 테스트 예시 결과

### 예시 1: 물품 배달
```
입력: "회의실로 커피 갖다줘"

🎯 작업 유형: ITEM_DELIVERY
✓ 신뢰도: 0.90
📋 추출된 필드:
  - location: 회의실
  - item: 커피
```

### 예시 2: 방문객 안내
```
입력: "방문객을 3층 회의실로 안내해줘"

🎯 작업 유형: GUIDE_GUEST
✓ 신뢰도: 0.92
📋 추출된 필드:
  - dest_location: 3층 회의실
```

### 예시 3: IoT 제어
```
입력: "온도 25도로 맞춰줘"

🎯 작업 유형: CONTROL_TEMPERATURE
✓ 신뢰도: 0.88
📋 추출된 필드:
  - device_type: THERMOSTAT
  - command: SET_VALUE
  - target_value: 25.0
```

### 예시 4: 회의실 예약
```
입력: "오후 2시에 회의실 예약해줘"

🎯 작업 유형: RESERVE_MEETING_ROOM
✓ 신뢰도: 0.85
📋 추출된 필드:
  - start_time: 14:00
```

## Proto 메시지 정의

```protobuf
// 요청 메시지
message NLRequest {
  string req_id = 1;        // 요청 ID
  string message = 2;       // 자연어 프롬프트
}

// 응답 메시지
message StructuredResponse {
  string req_id = 1;                    // 요청 ID
  TaskType task_type = 2;               // 작업 유형
  float confidence = 3;                 // 신뢰도
  StructuredMessage struct_msg = 4;     // 구조화된 메시지
  string raw_text = 5;                  // 원본 텍스트
}
```

## 주의사항

1. **LLM 모델 의존성**: Ollama의 `qwen3:4b-instruct-2507-q4_K_M` 모델이 필요합니다.
2. **신뢰도**: confidence 값이 낮은 경우 (< 0.7) 사용자에게 재확인을 요청하는 것이 좋습니다.
3. **에러 처리**: `task_type`이 `UNKNOWN`인 경우 적절한 피드백을 제공해야 합니다.
4. **필드 검증**: Main 서버에서 추출된 필드의 유효성을 검증해야 합니다.

## 문제 해결

### Proto 컴파일 오류
```bash
cd ai_server/grpc_impl
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. --pyi_out=. ai_services.proto
```

### Ollama 연결 실패
```bash
# Ollama 서비스 확인
ollama list

# 모델 재다운로드
ollama pull qwen3:4b-instruct-2507-q4_K_M
```

### gRPC 연결 실패
- LLM 서버가 실행 중인지 확인 (포트 50051)
- 방화벽 설정 확인
