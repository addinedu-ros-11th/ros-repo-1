


제공해주신 `node.py` 코드(오피스 로봇의 메인 실행 노드)에서 **직원 인식(Employee Verification) 및 QR 스캔 시나리오**는 주로 외부 비전(Vision) 시스템과의 연동 및 로컬 카메라를 통한 폴백(Fallback, 대체 수단) 로직으로 구성되어 있습니다.

통신 구조와 시나리오 로직을 중심으로 분석한 내용은 다음과 같습니다.

---

### 1. 통신 구조 (Communication Structure)
이 로봇은 ROS 2의 Pub/Sub 패턴을 사용하여 직원 인식 및 QR 검증을 수행합니다.

#### **[구독(Subscribe) - 입력]**
*   **`employee_verification_topic` (기본값: "employee_verification", `std_msgs/String`)**
    *   외부 AI 카메라/비전 노드로부터 **얼굴/사람 인식 결과**를 수신합니다.
    *   메시지 내부 페이로드: `person_type` (예: "employee"), `confidence` (신뢰도), `employee_id` 등을 포함.
*   **`qr_scan_image_topic` (기본값: "/camera/image_raw/compressed", `sensor_msgs/CompressedImage`)**
    *   QR 코드를 직접 스캔할 때 사용하는 **로봇 카메라의 압축 이미지 스트림**입니다.
*   **`commands` (`std_msgs/String`)**
    *   외부에서 명시적으로 `QR_SCAN` 액션 명령을 내릴 때 사용됩니다.

#### **[발행(Publish) - 출력]**
*   **`event` (`std_msgs/String`)**
    *   성공/실패 결과를 상위 관제 시스템에 보고합니다.
    *   발행 이벤트명: `EMPLOYEE_VERIFIED`(얼굴 인식 성공 시), `QR_SCANNED`(QR 성공 시), `QR_SCAN_FAILED`(QR 실패/타임아웃 시) 등.
*   **`display_topic` (`std_msgs/String`) & `led_topic` (`std_msgs/String`)**
    *   디스플레이 화면 텍스트("Hello, employee", "QR 코드를 인증해주세요", "인증 실패" 등)와 LED 상태(성공: 초록색 단일 점등, QR 대기: 빨간색 점멸)를 제어합니다.

---

### 2. 시나리오 로직 흐름 (Scenario Logic)

전체적인 시나리오는 **"1차 비전(얼굴) 인식 -> 실패 시 2차 QR 코드 인증 요구"**의 하이브리드 형태로 구성되어 있습니다.

#### **Step 1: 1차 비전 인식 (얼굴/형체 기반)**
1.  외부 노드에서 `employee_verification_topic`으로 데이터가 들어오면 `_on_employee_verification` 콜백이 실행됩니다.
2.  **신뢰도(Confidence) 검사:**
    *   `person_type == "employee"` 이고, `confidence >= employee_verification_min_confidence` 인 경우 **[인증 성공]**으로 간주합니다.
    *   로봇은 `EMPLOYEE_VERIFIED` 이벤트를 발행하고, LED를 초록색으로 켜며 인사말("Hello, employee")을 디스플레이에 띄웁니다 (`_handle_employee_verification`).
3.  **실패 조건 (QR 폴백 트리거):**
    *   `person_type`이 직원이 아니거나, 직원이지만 `confidence`가 기준치 미만인 경우 **[인증 실패]**로 간주하고 **QR 폴백 로직**(`_handle_employee_verification_non_employee`)으로 넘어갑니다.

#### **Step 2: 자동 QR 인증 모드 전환 (Fallback)**
1.  `employee_verification_qr_fallback_enabled` 파라미터가 True일 때만 작동합니다.
2.  현재 로봇이 주행 등 다른 액션을 수행 중이지 않고, 쿨다운(Cooldown) 시간이 지났다면 로봇의 상태를 **강제로 대기(`WAITING`)** 및 **QR 스캔 모드(`QR_SCAN` 액션)**로 전환합니다.
3.  LED를 **빨간색 점멸(Blink)**로 변경하고, 디스플레이에 **"QR 코드를 인증해주세요"**라고 띄웁니다.
4.  `_start_local_qr_scan()`을 호출하여 카메라 이미지 수신 및 분석 타이머를 시작합니다.

#### **Step 3: QR 코드 로컬 스캔 및 판독**
1.  로봇은 `qr_scan_poll_period_sec`(예: 0.2초) 주기로 카메라 이미지를 가져와 OpenCV(`cv2.QRCodeDetector`)로 판독합니다 (`_poll_local_qr_scan`).
2.  **방어 로직 (오인식 방지):**
    *   **Dwell Time:** QR 인식 모드 진입 후 최소 시간(`qr_scan_min_dwell_sec`)이 지나야 판독을 인정합니다.
    *   **Confirm Count:** 1회 인식으로 바로 통과시키지 않고, 동일한 QR 데이터가 연속으로 `qr_scan_confirm_count`(예: 2회) 만큼 인식되어야 유효한 데이터로 확정합니다.
3.  **판독 성공 시:**
    *   확정된 QR 데이터를 페이로드에 담아 `QR_SCANNED`(혹은 설정된 성공 이벤트) 이벤트를 발행하고 디스플레이에 "인증 성공"을 띄운 뒤 상태를 `IDLE`로 복귀시킵니다.

#### **Step 4: 타임아웃 및 실패 처리**
1.  `qr_scan_timeout_sec`(예: 15초) 동안 QR이 인식되지 않거나 읽기 실패가 지속되면 타임아웃 처리됩니다.
2.  `_handle_employee_verification_qr_timeout`이 호출되어 `QR_SCAN_FAILED` 이벤트를 발행합니다.
3.  디스플레이에 실패 원인에 따라 **"QR 코드가 보이지 않습니다"** 또는 **"QR 코드를 읽지 못했습니다"**를 출력하고 일반 대기 상태로 돌아갑니다.

---

### 3. 로직의 주요 특징 및 안정성 장치

*   **중복/반복 인식 방지 (Cooldown):**
    한 번 직원을 인식하거나 QR 인증 실패/성공 후에는 `employee_verification_cooldown_sec`(예: 5초) 동안 추가 인식을 무시하여 로봇이 계속 같은 사람에게 반응하거나 화면이 깜빡이는 것을 방지합니다.
*   **안전한 액션 개입 (Action Preemption):**
    QR 폴백이 발동되면 로봇은 임시로 현재 작업을 중단하고, 내부 액션 큐(`_current_action`)에 가상의 `QR_SCAN` 액션을 덮어씌워 시스템의 다른 부분(예: 명령어 무시 기능)이 로봇이 현재 스캔 중임을 알 수 있게 설계되었습니다.
*   **로컬 및 외부 시스템 유연성:**
    `qr_scan_local_enabled`를 통해 로봇 자체 보드(OpenCV)에서 QR을 처리할 수도 있고, 이 옵션을 끄면 단순히 타임아웃만 대기하며 외부 시스템이 데이터를 파싱해 명령(`on_success` 이벤트 발행)을 내려주기를 기다릴 수도 있는 유연한 구조입니다.