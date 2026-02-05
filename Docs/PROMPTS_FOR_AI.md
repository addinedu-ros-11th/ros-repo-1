# AI 코드 생성을 위한 프롬프트 가이드 (v2.0)

이 문서는 메인 서버와 분리된 **AI 서버** 및 **로봇**의 개발을 위한 프롬프트 모음입니다.

---

## 1. AI 추론 서버 개발 프롬프트 (gRPC 기반)

**목표:** 메인 서버로부터 gRPC 요청을 받고, 로봇으로부터 UDP 영상 스트림을 수신하여 추론 결과를 Push하는 서버를 개발합니다.

```text
Python과 gRPC를 사용하여 AI 추론 서버를 개발해줘.

### 주요 요구사항:
1.  **gRPC 서비스 구현**:
    -   `ai_inference.proto` 명세를 기반으로 서버를 구현해줘.
    -   `StreamInferenceResults`: 로봇으로부터 받은 영상을 분석하여 결과가 나올 때마다 클라이언트(메인 서버)에게 스트림으로 Push해야 해.
    -   `DetectObjects`, `RecognizeFaces`: 특정 이미지 ID에 대한 개별 요청에 응답해야 해.

2.  **UDP 영상 스트림 수신**:
    -   로봇이 직접 보내는 UDP 영상 스트림(포트 54321)을 비동기로 수신해야 해.
    -   수신된 프레임을 분석하여 `StreamInferenceResults`로 연결된 클라이언트들에게 결과를 전송해줘.

3.  **Mock AI 로직**:
    -   실제 모델 대신 `asyncio.sleep`을 사용하여 딜레이를 주고, 무작위 객체 인식 결과를 생성해줘.

### 기술 스택:
-   `grpcio`, `grpcio-tools`, `opencv-python`, `numpy`
```

---

## 2. 로봇 엣지 패키지 개발 프롬프트

**목표:** 메인 서버와는 ROS Bridge로, AI 서버와는 UDP로 통신하는 ROS 2 노드를 개발합니다.

```text
ROS 2 Jazzy를 사용하여 `communication_node`를 개발해줘.

### 주요 요구사항:
1.  **메인 서버 통신 (Control)**:
    -   `rosbridge_suite`를 통해 메인 서버와 연결해.
    -   `/robot/commands` 토픽을 구독하여 로봇의 동작을 제어하고, `/robot/status` 토픽을 발행하여 자신의 위치와 상태를 보고해.

2.  **AI 서버 통신 (Data)**:
    -   카메라 영상을 가져와서 AI 서버의 IP주소:54321로 UDP 스트리밍해.
    -   **중요**: 영상은 메인 서버가 아닌 AI 서버로만 직접 보내야 해.

### 기술 스택:
-   `rclpy`, `cv_bridge`, `opencv-python`
```