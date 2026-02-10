"""
웹캠으로 직원 얼굴 인식 테스트 (실시간)
"""

import cv2
import sys
from pathlib import Path
import numpy as np

# 프로젝트 루트를 path에 추가
sys.path.insert(0, str(Path(__file__).parent.parent))

from ai_server.services.vision_service import VisionService


def main():
    print("=" * 60)
    print("직원 얼굴 인식 웹캠 테스트 (실시간)")
    print("=" * 60)

    # VisionService 초기화
    print("\n1. Vision Service 초기화 중...")
    service = VisionService()
    service.initialize()
    print(f"   ✓ 직원 데이터베이스: {len(service._employee_face_db)}명 로드됨")

    # 웹캠 열기
    print("\n2. 웹캠 연결 중...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("   ✗ 웹캠을 열 수 없습니다.")
        return 1

    print("   ✓ 웹캠 연결 성공")
    print("\n" + "=" * 60)
    print("조작법:")
    print("  - 'q' 또는 ESC: 종료")
    print("  - 스페이스바: 현재 프레임 인식 결과 출력")
    print("=" * 60 + "\n")

    # 윈도우 생성 및 설정
    window_name = "직원 얼굴 인식 테스트 - Press Q to quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    print("윈도우 생성됨. 화면을 확인하세요!\n")

    frame_count = 0
    last_result = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("   ✗ 프레임을 읽을 수 없습니다.")
                break

            frame_count += 1
            display_frame = frame.copy()

            # 5프레임마다 얼굴 인식 실행
            if frame_count % 5 == 0:
                ok, buf = cv2.imencode(".jpg", frame)
                if ok:
                    try:
                        last_result = service.recognize_face(
                            image_id=f"webcam_frame_{frame_count}",
                            image_data=buf.tobytes(),
                        )
                    except Exception as e:
                        print(f"얼굴 인식 오류: {e}")
                        last_result = None

            # 결과를 화면에 오버레이
            if last_result:
                person_type = last_result["person_type"]
                confidence = last_result["confidence"]
                employee_id = last_result.get("employee_id", "N/A")

                # 결과에 따라 색상 변경
                if person_type == "Employee":
                    color = (0, 255, 0)  # 초록 - 직원
                    status = f"Employee: {employee_id}"
                elif person_type == "Guest":
                    color = (0, 165, 255)  # 주황 - 외부인
                    status = "Guest"
                else:
                    color = (0, 0, 255)  # 빨강 - 알 수 없음
                    status = "Unknown"

                # 상단에 정보 표시
                cv2.rectangle(display_frame, (10, 10), (630, 100), (0, 0, 0), -1)
                cv2.rectangle(display_frame, (10, 10), (630, 100), color, 2)

                cv2.putText(
                    display_frame,
                    status,
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    color,
                    2,
                )
                cv2.putText(
                    display_frame,
                    f"Confidence: {confidence:.2%}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

            # 프레임 번호 표시
            cv2.putText(
                display_frame,
                f"Frame: {frame_count}",
                (10, display_frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            # 화면 표시
            cv2.imshow(window_name, display_frame)

            # 첫 프레임에서 윈도우 위치 조정
            if frame_count == 1:
                cv2.moveWindow(window_name, 100, 100)

            # 키 입력 처리 (30ms 대기로 부드러운 재생)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q") or key == 27:  # 'q' 또는 ESC
                print("\n👋 테스트를 종료합니다.")
                break
            elif key == ord(" "):  # 스페이스바
                if last_result:
                    print("\n" + "=" * 60)
                    print("📊 현재 인식 결과")
                    print("=" * 60)
                    print(f"구분: {last_result['person_type']}")
                    print(f"신뢰도: {last_result['confidence']:.4f}")
                    if "employee_id" in last_result:
                        print(f"직원 ID: {last_result['employee_id']}")
                    print("=" * 60 + "\n")

    except KeyboardInterrupt:
        print("\n⚠️  사용자에 의해 중단되었습니다.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"\n📊 총 처리된 프레임: {frame_count}")
        print("✅ 종료 완료")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  사용자에 의해 중단되었습니다.")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ 오류 발생: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
