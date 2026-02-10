"""
Vision Service - YOLOv8n
객체 인식 및 얼굴 인식 기능을 처리하는 서비스 레이어
"""

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import face_recognition
import numpy as np

logger = logging.getLogger(__name__)


class VisionService:
    """
    YOLOv8n 모델을 사용한 컴퓨터 비전 서비스
    """

    def __init__(self, model_name: str = "yolov8n", model_path: str = None):
        """
        Vision 서비스 초기화

        Args:
            model_name: 모델 이름
            model_path: 모델 파일 경로
        """
        self.model_name = model_name
        self.model_path = model_path
        self.model = None
        self.employee_images_dir = (
            Path(__file__).resolve().parents[1] / "data" / "images"
        )
        self.encodings_cache_path = (
            Path(__file__).resolve().parents[1] / "data" / "employee_encodings.npy"
        )
        self.metadata_cache_path = (
            Path(__file__).resolve().parents[1] / "data" / "employee_metadata.json"
        )
        self.face_match_threshold = 0.3  # face_recognition distance threshold (lower = stricter) - 70% confidence
        self._employee_face_db: List[Dict[str, Any]] = []
        logger.info(f"Vision Service 초기화 중: {model_name}")

    def initialize(self):
        """
        모델을 로드하고 초기화
        실제 모델 로딩 코드는 나중에 구현
        """
        logger.info(f"Vision 모델 로딩 시작: {self.model_path}")
        # TODO: 실제 YOLOv8n 모델 로딩 로직 구현
        # from ultralytics import YOLO
        # self.model = YOLO(self.model_path)
        logger.info("Vision 모델 로딩 완료 (스텁)")
        self._load_employee_faces()

    def _load_employee_faces(self) -> None:
        """
        직원 얼굴 이미지 로딩 및 face_recognition 임베딩 생성
        캐시가 있고 유효하면 캐시에서 로드, 그렇지 않으면 재계산
        """
        self._employee_face_db = []
        if not self.employee_images_dir.exists():
            logger.warning("직원 이미지 폴더가 없습니다: %s", self.employee_images_dir)
            return

        # 현재 이미지 파일 목록
        image_files = list(self.employee_images_dir.glob("*.png")) + list(
            self.employee_images_dir.glob("*.jpg")
        )
        image_files += list(self.employee_images_dir.glob("*.jpeg"))
        if not image_files:
            logger.warning("직원 이미지가 없습니다: %s", self.employee_images_dir)
            return

        # 현재 파일 목록과 타임스탬프
        current_files = {str(f.name): f.stat().st_mtime for f in image_files}

        # 캐시 유효성 검사
        use_cache = False
        if self.encodings_cache_path.exists() and self.metadata_cache_path.exists():
            try:
                with open(self.metadata_cache_path, "r", encoding="utf-8") as f:
                    cached_metadata = json.load(f)

                # 파일 목록과 타임스탬프 비교
                if cached_metadata.get("files") == current_files:
                    use_cache = True
                    logger.info("인코딩 캐시 유효 - 캐시에서 로드")
            except Exception as e:
                logger.warning(f"캐시 메타데이터 읽기 실패: {e}")

        if use_cache:
            # 캐시에서 로드
            try:
                encodings = np.load(str(self.encodings_cache_path))
                with open(self.metadata_cache_path, "r", encoding="utf-8") as f:
                    metadata = json.load(f)

                employee_ids = metadata["employee_ids"]
                for i, employee_id in enumerate(employee_ids):
                    self._employee_face_db.append(
                        {"employee_id": employee_id, "encoding": encodings[i]}
                    )

                logger.info(
                    f"캐시에서 직원 얼굴 로딩 완료: {len(self._employee_face_db)}명"
                )
                return
            except Exception as e:
                logger.warning(f"캐시 로드 실패, 이미지에서 다시 계산: {e}")
                self._employee_face_db = []

        # 캐시 없거나 무효 - 이미지에서 계산
        logger.info("이미지에서 얼굴 인코딩 계산 중...")
        encodings_list = []
        employee_ids = []

        for image_path in sorted(image_files):
            image = face_recognition.load_image_file(str(image_path))
            face_encodings = face_recognition.face_encodings(image)

            if len(face_encodings) == 0:
                logger.warning(
                    "직원 이미지에서 얼굴을 찾을 수 없음: %s", image_path.name
                )
                continue

            # 가장 큰 얼굴의 인코딩 사용
            encoding = face_encodings[0]
            employee_id = image_path.stem

            self._employee_face_db.append(
                {"employee_id": employee_id, "encoding": encoding}
            )
            encodings_list.append(encoding)
            employee_ids.append(employee_id)
            logger.info("직원 얼굴 로딩: %s", employee_id)

        # 캐시 저장
        if encodings_list:
            try:
                # 인코딩 배열 저장
                encodings_array = np.array(encodings_list)
                np.save(str(self.encodings_cache_path), encodings_array)

                # 메타데이터 저장
                metadata = {
                    "employee_ids": employee_ids,
                    "files": current_files,
                    "count": len(employee_ids),
                }
                with open(self.metadata_cache_path, "w", encoding="utf-8") as f:
                    json.dump(metadata, f, ensure_ascii=False, indent=2)

                logger.info(f"인코딩 캐시 저장 완료: {len(encodings_list)}명")
            except Exception as e:
                logger.warning(f"캐시 저장 실패: {e}")

        logger.info("직원 얼굴 로딩 완료: %d명", len(self._employee_face_db))

    def _load_input_image(
        self, image_id: str, image_data: Optional[bytes]
    ) -> Optional[np.ndarray]:
        """
        요청 이미지 로딩
        """
        if image_data:
            data = np.frombuffer(image_data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is not None:
                return image

        if image_id:
            image_path = Path(image_id)
            if image_path.exists():
                return cv2.imread(str(image_path))
            candidate = self.employee_images_dir / image_id
            if candidate.exists():
                return cv2.imread(str(candidate))

        return None

    def detect_objects_from_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        OpenCV 프레임에서 직접 객체 인식

        Args:
            frame: OpenCV 프레임 (numpy array)

        Returns:
            객체 인식 결과
        """
        logger.debug(f"프레임 직접 처리: shape={frame.shape}")

        # TODO: 실제 YOLOv8n 추론 로직 구현
        # results = self.model(frame)
        # parsed = self._parse_yolo_results(results)

        # 스텁 응답
        return {
            "object_name": "person",
            "confidence": 0.95,
            "box": {"x": 100, "y": 150, "width": 200, "height": 300},
        }

    def detect_objects(self, image_id: str) -> Dict[str, Any]:
        """
        이미지에서 객체 인식

        Args:
            image_id: 이미지 ID

        Returns:
            객체 인식 결과
        """
        logger.info(f"객체 인식 요청: image_id={image_id}")
        # TODO: 실제 객체 인식 로직 구현
        # 1. image_id로 이미지 데이터 가져오기
        # 2. YOLO 모델로 추론
        # 3. 결과 파싱

        # 스텁 응답
        return {
            "object_name": "person",
            "confidence": 0.95,
            "box": {"x": 100, "y": 150, "width": 200, "height": 300},
        }

    def recognize_face(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        이미지에서 얼굴 인식 (face_recognition 라이브러리 사용)

        Args:
            image_id: 이미지 ID
            image_data: 이미지 바이트 데이터 (선택)

        Returns:
            얼굴 인식 결과
        """
        logger.info(f"얼굴 인식 요청: image_id={image_id}")

        # 입력 이미지 로딩
        image = self._load_input_image(image_id, image_data)
        if image is None:
            logger.warning("입력 이미지를 찾을 수 없습니다: %s", image_id)
            return {"person_type": "Unknown", "confidence": 0.0}

        # OpenCV BGR -> RGB 변환 (face_recognition은 RGB 사용)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 얼굴 인코딩 추출
        face_encodings = face_recognition.face_encodings(rgb_image)

        if len(face_encodings) == 0:
            logger.warning("입력 이미지에서 얼굴을 찾을 수 없음: %s", image_id)
            return {"person_type": "Unknown", "confidence": 0.0}

        # 첫 번째 얼굴 사용
        unknown_encoding = face_encodings[0]

        # 직원 데이터베이스가 비어있으면
        if not self._employee_face_db:
            logger.warning("직원 데이터베이스가 비어있음")
            return {"person_type": "Unknown", "confidence": 0.0}

        # 모든 직원 얼굴과 비교
        known_encodings = [emp["encoding"] for emp in self._employee_face_db]
        employee_ids = [emp["employee_id"] for emp in self._employee_face_db]

        # 얼굴 거리 계산 (낮을수록 유사)
        face_distances = face_recognition.face_distance(
            known_encodings, unknown_encoding
        )

        # 가장 가까운 얼굴 찾기
        best_match_index = int(np.argmin(face_distances))
        best_distance = float(face_distances[best_match_index])

        # 신뢰도 계산 (distance를 유사도로 변환: 1 - distance)
        confidence = 1.0 - best_distance

        logger.info(
            f"얼굴 매칭 결과: {employee_ids[best_match_index]}, "
            f"distance={best_distance:.4f}, confidence={confidence:.4f}"
        )

        # 임계값 이하면 직원으로 인식
        if best_distance <= self.face_match_threshold:
            return {
                "person_type": "Employee",
                "employee_id": employee_ids[best_match_index],
                "confidence": confidence,
            }

        # 임계값 초과면 외부인
        return {"person_type": "Guest", "confidence": confidence}

    def detect_multiple_objects(self, image_id: str) -> List[Dict[str, Any]]:
        """
        이미지에서 여러 객체 인식

        Args:
            image_id: 이미지 ID

        Returns:
            객체 인식 결과 리스트
        """
        logger.info(f"복수 객체 인식 요청: image_id={image_id}")
        # TODO: 실제 복수 객체 인식 로직 구현

        # 스텁 응답
        return [
            {
                "object_name": "person",
                "confidence": 0.95,
                "box": {"x": 100, "y": 150, "width": 200, "height": 300},
            },
            {
                "object_name": "chair",
                "confidence": 0.88,
                "box": {"x": 400, "y": 250, "width": 150, "height": 200},
            },
        ]
