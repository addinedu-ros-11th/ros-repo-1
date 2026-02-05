# Office Robot Service - 개발 가이드 (v3.0)

이 문서는 Office Robot Service 프로젝트의 개선된 구조와 gRPC 기반의 AI 연동 방식을 설명합니다.

---

## 1. 프로젝트 아키텍처 개요

본 프로젝트는 다음과 같은 애플리케이션들로 구성됩니다.

-   **`main_server/`**: 컨트롤 타워 역할을 하는 메인 서버 (FastAPI).
-   **`robot/`**: 로봇 위에서 구동되는 ROS 2 패키지.
-   **AI Server (별도)**: GPU 기반의 AI 추론 전용 서버.

## 2. 메인 서버 실행 방법 (`main_server/`)

### 필수 환경
- Python 3.8+
- 주요 패키지: `fastapi`, `uvicorn`, `aiomysql`, `roslibpy`, `grpcio`, `grpcio-tools`, `python-dotenv`
- **설치**: `pip install -r requirements.txt`

### 실행
1.  프로젝트 루트의 `.env` 파일을 설정합니다. (DB 및 ROS 브리지 정보)
2.  서버 시작: `uvicorn main_server.app:app --reload`
3.  접속 주소:
    - **관리자 대시보드**: `/web/admin`
    - **직원용 앱**: `/web/employee`
    - **API 문서**: `/docs`

### 실행 순서 (Startup Flow)
1.  `Database.initialize()`: 설정된 정보를 바탕으로 DB 연결 풀 생성.
2.  `container.services()`: DI 컨테이너를 통한 서비스 초기화.
3.  **`ROSBridge` 시작**: 로봇과의 실시간 통신 활성화.
4.  **`AI Stream` 구독**: AI 서버로부터 실시간 추론 결과를 받기 위한 gRPC 스트림 연결.

---

## 3. 디렉토리 구조 및 주요 모듈

### `main_server/`
-   **`config.py`**: `.env` 및 중앙 집중식 설정 관리.
-   **`infrastructure/communication/`**
    -   `ros_bridge.py`: `roslibpy`를 사용한 실제 로봇과의 통신 구현.
-   **`core_layer/ai_inference/`**
    -   `grpc_inference_client.py`: AI 서버와의 gRPC 통신 클라이언트 (Streaming 지원).
-   **`infrastructure/grpc/`**: gRPC 프로토콜 정의 및 자동 생성된 파이썬 코드.

---

## 4. API 및 프로토콜 명세

### A. 메인 서버 <-> 로봇 (ROS Bridge)
-   **제어**: `/robot/commands` (JSON) - 액션 시퀀스 전달.
-   **상태**: `/robot/status` (JSON) - 위치 및 배터리 정보 수신.

### B. 메인 서버 <-> AI 서버 (gRPC)
-   **Proto**: `main_server/infrastructure/grpc/ai_inference.proto`
-   **서비스**:
    -   `StreamInferenceResults`: 실시간 추론 결과 구독 (Server Streaming).
    -   `DetectObjects`: 개별 이미지 객체 인식 요청 (Unary).
    -   `RecognizeFaces`: 개별 이미지 얼굴 인식 요청 (Unary).

---

## 5. 구현 로드맵 현황

-   [x] **중앙 집중식 설정 관리**: `.env` 기반 `config.py` 구현 완료.
-   [x] **실제 로봇 통신 구현**: `ROSBridge` 연동 완료.
-   [x] **AI 추론 스트리밍**: gRPC Server-side Streaming 클라이언트 구현 완료.
-   [ ] **배차 알고리즘 고도화**: 단순 거리 기반에서 상태 가중치 모델로 발전 필요.
-   [ ] **AI 서버 (별도 저장소)**: 실제 GPU 추론 로직 구현 필요.
