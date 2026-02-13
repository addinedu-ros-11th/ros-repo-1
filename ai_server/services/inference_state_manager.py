"""
Inference State Manager
로봇별 추론 상태(모델 타입별 활성/비활성)를 관리하는 모듈
메인서버가 UpdateInferenceState RPC로 상태를 변경하면 이 매니저가 추적
"""

import logging
import threading
import time
from collections import defaultdict
from typing import Dict, List, Set

logger = logging.getLogger(__name__)


class InferenceStateManager:
    """
    로봇별 추론 상태를 thread-safe 하게 관리.

    지원 모델 타입:
        - EMPLOYEE : 얼굴 인식 (유휴 모드)
        - SNACK    : 상품/간식 감지 (배달 시나리오)
        - OBSTACLE : 장애물 감지 (주행 모드 - 사람/의자/화분)
    """

    VALID_MODEL_TYPES = {"EMPLOYEE", "SNACK", "OBSTACLE"}

    def __init__(self):
        # { robot_id: { model_type: is_active } }
        self._states: Dict[str, Dict[str, bool]] = defaultdict(dict)
        self._lock = threading.Lock()
        logger.info("InferenceStateManager 초기화 완료")

    def update_state(self, robot_id: str, model_type: str, is_active: bool) -> bool:
        """
        특정 로봇의 특정 모델 추론 상태를 업데이트.

        Args:
            robot_id: 로봇 식별자
            model_type: 모델 타입 (EMPLOYEE / SNACK / OBSTACLE)
            is_active: 활성화 여부

        Returns:
            성공 여부
        """
        model_type = model_type.upper()
        if model_type not in self.VALID_MODEL_TYPES:
            logger.warning(f"알 수 없는 모델 타입: {model_type}")
            return False

        with self._lock:
            prev = self._states[robot_id].get(model_type, False)
            self._states[robot_id][model_type] = is_active

            # 비활성화할 때 clean-up
            if not is_active and model_type in self._states[robot_id]:
                self._states[robot_id][model_type] = False

        action = "활성화" if is_active else "비활성화"
        if prev != is_active:
            logger.info(
                f"추론 상태 변경: robot={robot_id}, model={model_type}, {action}"
            )
        return True

    def get_active_models(self, robot_id: str) -> Set[str]:
        """
        특정 로봇에 대해 현재 활성화된 모델 타입 목록 반환.

        Args:
            robot_id: 로봇 식별자

        Returns:
            활성화된 모델 타입 set (예: {"EMPLOYEE", "OBSTACLE"})
        """
        with self._lock:
            states = self._states.get(robot_id, {})
            return {mt for mt, active in states.items() if active}

    def is_active(self, robot_id: str, model_type: str) -> bool:
        """
        특정 로봇의 특정 모델이 활성 상태인지 확인.
        """
        with self._lock:
            return self._states.get(robot_id, {}).get(model_type.upper(), False)

    def has_any_active(self, robot_id: str) -> bool:
        """
        특정 로봇에 활성화된 추론이 하나라도 있는지 확인.
        """
        with self._lock:
            states = self._states.get(robot_id, {})
            return any(states.values())

    def has_any_active_globally(self) -> bool:
        """
        어떤 로봇이든 활성화된 추론이 있는지 확인.
        """
        with self._lock:
            for robot_states in self._states.values():
                if any(robot_states.values()):
                    return True
            return False

    def get_all_states(self) -> Dict[str, Dict[str, bool]]:
        """
        전체 상태 스냅샷 반환 (디버깅/로깅용).
        """
        with self._lock:
            return {rid: dict(states) for rid, states in self._states.items()}

    def clear_robot(self, robot_id: str):
        """
        특정 로봇의 모든 추론 상태를 초기화.
        """
        with self._lock:
            if robot_id in self._states:
                del self._states[robot_id]
                logger.info(f"로봇 상태 초기화: {robot_id}")

    def clear_all(self):
        """
        모든 로봇의 추론 상태 초기화.
        """
        with self._lock:
            self._states.clear()
            logger.info("전체 추론 상태 초기화")
