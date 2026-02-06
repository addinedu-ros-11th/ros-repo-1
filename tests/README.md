# Tests

이 디렉토리에는 프로젝트의 모든 테스트 파일이 포함되어 있습니다.

## 구조

```
tests/
├── test_communication.py              # 통신 테스트
├── test_entity_extraction_simple.py   # LLM 엔티티 추출 단독 테스트
├── test_entity_extraction_grpc.py     # LLM 엔티티 추출 gRPC 테스트
└── ai_server/                         # AI 서버 관련 테스트
    ├── test_client.py                 # AI 서버 통합 클라이언트 테스트
    ├── test_client_separated.py       # 분리된 서버 클라이언트 테스트
    └── test_udp_video.sh              # UDP 비디오 스트리밍 테스트
```

## 실행 방법

### LLM 엔티티 추출 테스트

**간단한 테스트 (추천)**:
```bash
python tests/test_entity_extraction_simple.py
```

**gRPC 테스트**:
```bash
# 터미널 1: LLM 서버 시작
python -m ai_server.llm_server

# 터미널 2: 테스트 실행
python tests/test_entity_extraction_grpc.py
```

### 통신 테스트
```bash
python tests/test_communication.py
```

### AI 서버 테스트
```bash
# 통합 클라이언트
python tests/ai_server/test_client.py

# 분리된 서버
python tests/ai_server/test_client_separated.py

# UDP 비디오
bash tests/ai_server/test_udp_video.sh
```

## 참고
자세한 사용법은 각 테스트 파일의 docstring을 참고하거나 [ENTITY_EXTRACTION_GUIDE.md](../ENTITY_EXTRACTION_GUIDE.md)를 확인하세요.
