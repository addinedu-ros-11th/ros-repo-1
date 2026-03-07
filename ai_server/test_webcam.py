"""
웹캠을 사용한 YOLO 모델 실시간 테스트 스크립트
obstacle.pt 모델 하나만 사용합니다.
"""

import cv2
import numpy as np
from pathlib import Path
import sys

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ultralytics 패키지가 설치되지 않았습니다.")
    print("설치 명령: pip install ultralytics")
    sys.exit(1)


class SingleModelWebcamTest:
    """단일 YOLO 모델을 사용한 웹캠 테스트"""

    def __init__(self, obstacle_model_path: str):
        """
        Args:
            obstacle_model_path: 장애물 감지 모델 경로
        """
        self.obstacle_model_path = Path(obstacle_model_path)

        # 모델 로드
        print("🔄 모델 로딩 중...")
        try:
            self.obstacle_model = YOLO(str(self.obstacle_model_path))
            print(f"✅ 장애물 모델 로드 완료: {self.obstacle_model_path.name}")
        except Exception as e:
            print(f"❌ 장애물 모델 로드 실패: {e}")
            sys.exit(1)

        # 웹캠 초기화
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("❌ 웹캠을 열 수 없습니다.")
            sys.exit(1)

        print("✅ 웹캠 초기화 완료")

        # 색상 설정 (BGR)
        self.obstacle_color = (0, 0, 255)  # 빨강 - 장애물
        # 제품 모델 없음, color 설정 생략

    def draw_detections(self, frame, results, color, label_prefix):
        """
        감지 결과를 프레임에 그리기

        Args:
            frame: 원본 프레임
            results: YOLO 결과
            color: 바운딩 박스 색상
            label_prefix: 라벨 접두사
        """
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 바운딩 박스 좌표
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 신뢰도와 클래스
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = result.names[class_id]

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # 라벨 텍스트
                label = f"{class_name} {confidence:.2f}"

                # 라벨 배경
                (text_width, text_height), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
                )
                cv2.rectangle(
                    frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1
                )

                # 라벨 텍스트
                cv2.putText(
                    frame,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

        return frame

    def run(self, confidence_threshold=0.5):
        """
        웹캠 테스트 실행

        Args:
            confidence_threshold: 감지 신뢰도 임계값
        """
        print("\n" + "=" * 60)
        print("🎥 웹캠 테스트 시작")
        print("=" * 60)
        print(f"⚠️  장애물 모델: {self.obstacle_model_path.name} (빨강)")
        print(f"🎯 신뢰도 임계값: {confidence_threshold}")
        print("\n조작법:")
        print("  - 'q' 또는 ESC: 종료")
        print("  - 's': 현재 프레임 스크린샷 저장")
        print("=" * 60 + "\n")

        frame_count = 0
        screenshot_count = 0

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ 프레임을 읽을 수 없습니다.")
                    break

                frame_count += 1

                # 모델로 추론 실행
                obstacle_results = self.obstacle_model(
                    frame, conf=confidence_threshold, verbose=False
                )

                # 결과를 프레임에 그리기
                frame = self.draw_detections(
                    frame, obstacle_results, self.obstacle_color, "OBS"
                )

                # 감지된 객체 수 표시
                obstacle_count = sum(len(r.boxes) for r in obstacle_results)

                # 정보 오버레이
                info_text = [
                    f"Frame: {frame_count}",
                    f"Obstacles: {obstacle_count}",
                ]

                y_offset = 30
                for i, text in enumerate(info_text):
                    cv2.putText(
                        frame,
                        text,
                        (10, y_offset + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                # 프레임 표시
                cv2.imshow("YOLO Webcam Test - Press Q to quit", frame)

                # 키 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:  # 'q' 또는 ESC
                    print("\n👋 테스트를 종료합니다.")
                    break
                elif key == ord("s"):  # 스크린샷
                    screenshot_count += 1
                    filename = f"screenshot_{screenshot_count}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"📸 스크린샷 저장: {filename}")

        except KeyboardInterrupt:
            print("\n⚠️  사용자에 의해 중단되었습니다.")
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            print(f"\n📊 총 처리된 프레임: {frame_count}")
            print("✅ 종료 완료")


def main():
    """메인 함수"""
    import argparse

    parser = argparse.ArgumentParser(description="YOLO webcam single-model test")
    parser.add_argument(
        "--model", default="ai_server/models/obstacle.pt", help="obstacle 모델 경로"
    )
    parser.add_argument("--conf", type=float, default=0.5, help="신뢰도 임계값")
    args = parser.parse_args()

    tester = SingleModelWebcamTest(args.model)
    tester.run(confidence_threshold=args.conf)
    # 모델 경로 설정
    base_path = Path(__file__).parent / "models"
    obstacle_model = base_path / "obstacle.pt"
    product_model = base_path / "product.pt"

    # 모델 파일 존재 확인
    if not obstacle_model.exists():
        print(f"❌ 장애물 모델을 찾을 수 없습니다: {obstacle_model}")
        sys.exit(1)

    if not product_model.exists():
        print(f"❌ 제품 모델을 찾을 수 없습니다: {product_model}")
        sys.exit(1)

    # 테스트 실행
    tester = DualModelWebcamTest(
        obstacle_model_path=str(obstacle_model), product_model_path=str(product_model)
    )

    # 신뢰도 임계값 설정 (필요시 조정)
    confidence_threshold = 0.5
    tester.run(confidence_threshold=confidence_threshold)


if __name__ == "__main__":
    main()
