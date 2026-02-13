"""
Vision Service
YOLO 객체 인식 (상품/장애물) 및 얼굴 인식 기능을 처리하는 서비스 레이어

모델 타입:
  - SNACK    : product.pt (배달 시나리오 - 간식/상품 감지)
  - OBSTACLE : obstacle.pt (주행 모드 - 사람/의자/화분 등 장애물 감지)
  - EMPLOYEE : face_recognition 라이브러리 (유휴 모드 - 직원/외부인 판별)
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
    YOLO + face_recognition 기반 비전 서비스.
    상품 감지, 장애물 감지, 얼굴 인식 세 기능을 제공.
    """

    def __init__(
        self,
        product_model_path: str = None,
        obstacle_model_path: str = None,
        face_match_threshold: float = 0.3,
        yolo_confidence: float = 0.5,
    ):
        """
        Vision 서비스 초기화

        Args:
            product_model_path: 상품 감지 YOLO 모델 경로 (product.pt)
            obstacle_model_path: 장애물 감지 YOLO 모델 경로 (obstacle.pt)
            face_match_threshold: 얼굴 매칭 임계값 (낮을수록 엄격)
            yolo_confidence: YOLO 신뢰도 임계값
        """
        self.product_model_path = product_model_path
        self.obstacle_model_path = obstacle_model_path
        self.yolo_confidence = yolo_confidence

        # YOLO 모델 인스턴스
        self._product_model = None
        self._obstacle_model = None

        # 얼굴 인식 설정
        self.employee_images_dir = (
            Path(__file__).resolve().parents[1] / "data" / "images"
        )
        self.encodings_cache_path = (
            Path(__file__).resolve().parents[1] / "data" / "employee_encodings.npy"
        )
        self.metadata_cache_path = (
            Path(__file__).resolve().parents[1] / "data" / "employee_metadata.json"
        )
        self.face_match_threshold = face_match_threshold
        self._employee_face_db: List[Dict[str, Any]] = []

        logger.info("VisionService 초기화")

    def initialize(self):
        """
        모든 모델을 로드하고 초기화.
        """
        self._load_yolo_models()
        self._load_employee_faces()
        logger.info("VisionService 초기화 완료")

    # ========================================
    # YOLO 모델 로딩
    # ========================================

    def _load_yolo_models(self):
        """
        YOLO 모델 로딩 (product.pt, obstacle.pt)
        ultralytics 패키지 사용
        """
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error(
                "ultralytics 패키지가 설치되지 않았습니다. pip install ultralytics"
            )
            return

        # 상품 감지 모델
        if self.product_model_path and Path(self.product_model_path).exists():
            try:
                self._product_model = YOLO(self.product_model_path)
                logger.info(f"상품 감지 모델 로딩 완료: {self.product_model_path}")
            except Exception as e:
                logger.error(f"상품 감지 모델 로딩 실패: {e}")
        else:
            logger.warning(f"상품 감지 모델 파일 없음: {self.product_model_path}")

        # 장애물 감지 모델
        if self.obstacle_model_path and Path(self.obstacle_model_path).exists():
            try:
                self._obstacle_model = YOLO(self.obstacle_model_path)
                logger.info(f"장애물 감지 모델 로딩 완료: {self.obstacle_model_path}")
            except Exception as e:
                logger.error(f"장애물 감지 모델 로딩 실패: {e}")
        else:
            logger.warning(f"장애물 감지 모델 파일 없음: {self.obstacle_model_path}")

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
        요청 이미지 로딩 (bytes 또는 파일 경로)
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

    # ========================================
    # YOLO 추론 메서드 (실제 구현)
    # ========================================

    def _parse_yolo_results(self, results) -> List[Dict[str, Any]]:
        """
        YOLO 추론 결과를 파싱하여 딕셔너리 리스트로 변환.

        Args:
            results: ultralytics YOLO 추론 결과

        Returns:
            감지된 객체 리스트
        """
        detections = []
        if not results or len(results) == 0:
            return detections

        result = results[0]  # 첫 번째 이미지 결과
        if result.boxes is None or len(result.boxes) == 0:
            return detections

        for box in result.boxes:
            conf = float(box.conf[0])
            if conf < self.yolo_confidence:
                continue

            cls_id = int(box.cls[0])
            class_name = result.names.get(cls_id, f"class_{cls_id}")

            # xyxy → x, y, width, height
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(
                {
                    "object_name": class_name,
                    "confidence": round(conf, 4),
                    "box": {
                        "x": int(x1),
                        "y": int(y1),
                        "width": int(x2 - x1),
                        "height": int(y2 - y1),
                    },
                }
            )

        return detections

    def detect_products_from_frame(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        프레임에서 상품/간식 감지 (product.pt)

        Args:
            frame: OpenCV BGR 프레임

        Returns:
            감지된 상품 리스트
        """
        if self._product_model is None:
            logger.debug("상품 감지 모델이 로드되지 않음")
            return []

        try:
            results = self._product_model(frame, verbose=False)
            detections = self._parse_yolo_results(results)
            if detections:
                logger.debug(f"상품 감지: {len(detections)}개")
            return detections
        except Exception as e:
            logger.error(f"상품 감지 추론 실패: {e}")
            return []

    def detect_obstacles_from_frame(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        프레임에서 장애물 감지 (obstacle.pt — 사람/의자/화분 등)

        Args:
            frame: OpenCV BGR 프레임

        Returns:
            감지된 장애물 리스트
        """
        if self._obstacle_model is None:
            logger.debug("장애물 감지 모델이 로드되지 않음")
            return []

        try:
            results = self._obstacle_model(frame, verbose=False)
            detections = self._parse_yolo_results(results)
            if detections:
                logger.debug(f"장애물 감지: {len(detections)}개")
            return detections
        except Exception as e:
            logger.error(f"장애물 감지 추론 실패: {e}")
            return []

    def recognize_face_from_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        프레임에서 직접 얼굴 인식 (face_recognition 라이브러리 사용)

        Args:
            frame: OpenCV BGR 프레임

        Returns:
            얼굴 인식 결과 {"person_type", "employee_id"?, "confidence"}
        """
        if not self._employee_face_db:
            return {"person_type": "Unknown", "confidence": 0.0}

        try:
            # BGR → RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # 성능을 위해 프레임 축소 (1/2)
            small_frame = cv2.resize(rgb_frame, (0, 0), fx=0.5, fy=0.5)

            # 얼굴 위치 + 인코딩 추출
            face_locations = face_recognition.face_locations(small_frame, model="hog")
            if not face_locations:
                return {"person_type": "Unknown", "confidence": 0.0}

            face_encodings = face_recognition.face_encodings(
                small_frame, face_locations
            )
            if not face_encodings:
                return {"person_type": "Unknown", "confidence": 0.0}

            # 첫 번째 얼굴에 대해 매칭
            unknown_encoding = face_encodings[0]
            known_encodings = [emp["encoding"] for emp in self._employee_face_db]
            employee_ids = [emp["employee_id"] for emp in self._employee_face_db]

            face_distances = face_recognition.face_distance(
                known_encodings, unknown_encoding
            )

            best_match_index = int(np.argmin(face_distances))
            best_distance = float(face_distances[best_match_index])
            confidence = round(1.0 - best_distance, 4)

            if best_distance <= self.face_match_threshold:
                return {
                    "person_type": "Employee",
                    "employee_id": employee_ids[best_match_index],
                    "confidence": confidence,
                }

            return {"person_type": "Guest", "confidence": confidence}

        except Exception as e:
            logger.error(f"프레임 얼굴 인식 실패: {e}")
            return {"person_type": "Unknown", "confidence": 0.0}

    # ========================================
    # gRPC 단건 요청용 (기존 호환)
    # ========================================

    def detect_objects(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        이미지에서 객체 인식 (product 모델 사용, 단건 요청)
        """
        image = self._load_input_image(image_id, image_data)
        if image is None:
            logger.warning(f"이미지 로드 실패: {image_id}")
            return {
                "object_name": "none",
                "confidence": 0.0,
                "box": {"x": 0, "y": 0, "width": 0, "height": 0},
            }

        detections = self.detect_products_from_frame(image)
        if detections:
            return detections[0]
        return {
            "object_name": "none",
            "confidence": 0.0,
            "box": {"x": 0, "y": 0, "width": 0, "height": 0},
        }

    def recognize_face(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        이미지에서 얼굴 인식 (단건 요청)
        """
        logger.info(f"얼굴 인식 요청: image_id={image_id}")
        image = self._load_input_image(image_id, image_data)
        if image is None:
            logger.warning("입력 이미지를 찾을 수 없습니다: %s", image_id)
            return {"person_type": "Unknown", "confidence": 0.0}
        return self.recognize_face_from_frame(image)

    def detect_multiple_objects(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> List[Dict[str, Any]]:
        """
        이미지에서 여러 객체 인식 (obstacle 모델 사용, 단건 요청)
        """
        image = self._load_input_image(image_id, image_data)
        if image is None:
            logger.warning(f"이미지 로드 실패: {image_id}")
            return []

        return self.detect_obstacles_from_frame(image)
