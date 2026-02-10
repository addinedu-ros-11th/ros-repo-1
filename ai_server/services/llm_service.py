"""
LLM Service - Qwen3-4B with Ollama
LLM 관련 기능을 처리하는 서비스 레이어
"""

import json
import logging
import re
from typing import Any, Dict, List

import ollama

logger = logging.getLogger(__name__)

ENTITY_SYSTEM_PROMPT = """당신은 사무실 로봇을 위한 명령어 파서입니다.
사용자의 요청에서 다음 정보만 추출하세요:
1. location (장소): 예) 회의실, 로비, 사무실, 301호, A동 등
2. item (물품): 예) 커피, 서류, 노트북, 상자 등

반드시 다음 JSON 형식으로만 답변하세요:
{"location": "장소이름 또는 null", "item": "물품이름 또는 null"}

중요:
- 장소나 물품이 없으면 null로 표시
- JSON 형식 외의 다른 텍스트는 절대 포함하지 마세요
- 추가 설명이나 인사말도 하지 마세요"""

PARSE_SYSTEM_PROMPT = """당신은 사무실 로봇 서비스를 위한 자연어 명령 파서입니다.
사용자의 요청을 분석하여 작업 유형과 관련 정보를 추출하세요.

작업 유형 (task_type):
1. SNACK_DELIVERY: 간식 배달 (예: "커피 갖다줘", "간식 가져와줘")
2. ITEM_DELIVERY: 물품 배달 (예: "서류 전달해줘", "노트북 갖다줘")
3. PICKUP_ITEM: 물품 수거 (예: "박스 가져와", "물건 수거해줘")
4. GUIDE_GUEST: 방문객 안내 (예: "손님 안내해줘", "방문자 가이드")
5. NAVIGATE_TO_LOCATION: 위치로 이동 (예: "회의실로 가줘", "로비로 이동")
6. FOLLOW_PERSON: 사람 따라가기 (예: "나를 따라와", "김철수씨 따라가")
7. CALL_ROBOT: 로봇 호출 (예: "로봇 불러줘", "여기로 와")
8. RETURN_TO_BASE: 복귀 명령 (예: "돌아가", "충전하러 가")
9. CANCEL_TASK: 작업 취소 (예: "취소해줘", "그만해")
10. PAUSE_TASK: 일시정지 (예: "잠깐 멈춰", "대기해")
11. RESUME_TASK: 재개 (예: "다시 시작", "계속해")
12. CONTROL_LIGHT: 조명 제어 (예: "불 켜줘", "조명 꺼줘")
13. CONTROL_TEMPERATURE: 온도 제어 (예: "온도 올려줘", "따뜻하게 해줘")
14. CONTROL_AC: 에어컨 제어 (예: "에어컨 켜줘", "냉방 시작")
15. CONTROL_DOOR: 문 제어 (예: "문 잠가줘", "문 열어줘")
16. QUERY_ROBOT_STATUS: 로봇 상태 조회 (예: "로봇 상태는?", "배터리 얼마나 남았어?")
17. QUERY_LOCATION: 위치 조회 (예: "회의실이 어디야?", "로비 어디에 있어?")
18. QUERY_AVAILABILITY: 가용성 조회 (예: "회의실 비어있어?", "자리 있어?")
19. FIND_PERSON: 사람 찾기 (예: "김철수 어디있어?", "박영희씨 찾아줘")
20. FIND_ITEM: 물건 찾기 (예: "내 노트북 어디있어?", "서류 찾아줘")
21. RESERVE_MEETING_ROOM: 회의실 예약 (예: "회의실 예약해줘", "2시에 회의실 잡아줘")
22. CANCEL_RESERVATION: 예약 취소 (예: "회의실 예약 취소", "예약 해제해줘")
23. CHECK_ROOM_STATUS: 회의실 상태 확인 (예: "회의실 상태 보여줘", "어느 방 사용중이야?")
24. PATROL_AREA: 순찰 (예: "2층 순찰해줘", "사무실 한 바퀴 돌아")
25. MONITOR_ENVIRONMENT: 환경 모니터링 (예: "환경 체크해줘", "온도 습도 확인해")
26. GENERAL_QUESTION: 일반 질문 (예: "오늘 날씨 어때?", "회사 규정 알려줘")
27. GREETING: 인사 (예: "안녕", "반가워")

추출할 필드 (없으면 null):
- location: 위치/장소
- item: 물품명
- person_name: 사람 이름
- source_location: 출발지
- dest_location: 목적지
- quantity: 수량 (숫자)
- device_type: IoT 장치 (LIGHT/THERMOSTAT/AIR_CONDITIONER/DOOR_LOCK)
- command: IoT 명령 (TURN_ON/TURN_OFF/SET_VALUE/LOCK/UNLOCK)
- target_value: 목표 값 (온도 등, 숫자)
- room_id: 방 ID
- meeting_room_id: 회의실 ID
- start_time: 시작 시간
- end_time: 종료 시간
- attendee_count: 참석자 수 (숫자)
- area: 구역
- waypoints: 경유지 목록 (배열)
- keywords: 키워드 목록 (배열)
- message: 일반 메시지

반드시 다음 JSON 형식으로만 답변하세요:
{
    "task_type": "작업유형",
    "confidence": 0.9,
    "fields": {
        "field_name": "value"
    }
}

중요:
- 아래 허용된 필드만 사용하세요 (다른 키 사용 금지)
    location, item, person_name, source_location, dest_location,
    quantity, device_type, command, target_value, room_id,
    meeting_room_id, start_time, end_time, attendee_count,
    area, waypoints, keywords, message
- 해당하지 않는 필드는 포함하지 마세요
- JSON 외의 다른 텍스트는 절대 포함하지 마세요
- 배열 필드는 ["item1", "item2"] 형식으로"""


class LLMService:
    """
    LLM(Qwen3-4B) 모델을 사용한 자연어 처리 서비스
    Ollama를 통해 로컬 모델 실행
    """

    def __init__(
        self, model_name: str = "qwen3:4b-instruct-2507-q4_K_M", model_path: str = None
    ):
        """
        LLM 서비스 초기화

        Args:
            model_name: Ollama 모델 이름
            model_path: 모델 파일 경로 (Ollama 사용 시 무시됨)
        """
        self.model_name = model_name
        self.model_path = model_path
        self.model = None
        logger.info(f"LLM Service 초기화 중: {model_name}")

    def initialize(self):
        """
        모델을 로드하고 초기화
        Ollama는 필요 시 자동으로 모델을 로드함
        """
        logger.info(f"Ollama LLM 모델 준비: {self.model_name}")
        try:
            # Ollama 모델이 실행 가능한지 확인
            response = ollama.chat(
                model=self.model_name, messages=[{"role": "user", "content": "test"}]
            )
            logger.info("Ollama LLM 모델 로딩 완료")
        except Exception as e:
            logger.error(f"Ollama 모델 초기화 실패: {e}")
            raise

    def generate_text(
        self, prompt: str, max_length: int = 100, temperature: float = 0.7
    ) -> str:
        """
        텍스트 생성

        Args:
            prompt: 입력 프롬프트
            max_length: 최대 생성 길이
            temperature: 생성 온도 (0.0~1.0)

        Returns:
            생성된 텍스트
        """
        logger.info(f"텍스트 생성 요청: {prompt[:50]}...")

        try:
            response = ollama.generate(
                model=self.model_name,
                prompt=prompt,
                options={"temperature": temperature, "num_predict": max_length},
            )
            result = response["response"]
            logger.info(f"텍스트 생성 완료: {len(result)} 문자")
            return result
        except Exception as e:
            logger.error(f"텍스트 생성 실패: {e}")
            raise

    def chat(self, messages: List[Dict[str, str]]) -> str:
        """
        대화형 응답 생성

        Args:
            messages: 대화 히스토리 [{"role": "user"/"assistant", "content": "..."}]

        Returns:
            생성된 응답
        """
        logger.info(f"대화 요청: {len(messages)} 메시지")

        result = self._chat(messages)
        logger.info("대화 응답 생성 완료")
        return result

    def analyze_intent(self, text: str) -> Dict[str, Any]:
        """
        텍스트에서 의도 분석

        Args:
            text: 분석할 텍스트

        Returns:
            의도 분석 결과
        """
        logger.info(f"의도 분석 요청: {text[:50]}...")
        # TODO: 실제 의도 분석 로직 구현
        return {"intent": "unknown", "confidence": 0.0, "entities": []}

    def _sanitize_json_text(self, text: str) -> str:
        """
        LLM 응답을 JSON 파싱 가능하도록 보정
        """
        # 코드 블록 제거
        cleaned = re.sub(r"```json\s*|\s*```", "", text).strip()
        # 리스트 항목처럼 출력된 잘못된 접두어 제거
        cleaned = re.sub(r"^\s*-\s*\"", '"', cleaned, flags=re.MULTILINE)
        # 불필요한 trailing comma 제거
        cleaned = re.sub(r",\s*\}", "}", cleaned)
        cleaned = re.sub(r",\s*\]", "]", cleaned)
        return cleaned

    def _parse_json_response(self, text: str) -> Dict[str, Any]:
        """
        LLM 응답을 JSON으로 파싱
        """
        cleaned = self._sanitize_json_text(text)
        return json.loads(cleaned)

    def _normalize_null_value(self, value: Any) -> Any:
        """
        null/빈 문자열을 None으로 표준화
        """
        if value is None:
            return None
        if isinstance(value, str) and value.strip().lower() in {"null", ""}:
            return None
        return value

    def _chat(self, messages: List[Dict[str, str]], temperature: float = 0.7) -> str:
        """
        Ollama chat 호출
        """
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=messages,
                options={"temperature": temperature},
            )
            return response["message"]["content"].strip()
        except Exception as e:
            logger.error(f"대화 처리 실패: {e}")
            raise

    def _normalize_task_type(self, task_type: str) -> str:
        """
        task_type 표준화
        """
        if not task_type:
            return "UNKNOWN"
        upper = str(task_type).strip().upper()
        alias_map = {
            "QUERY_ROOM_STATUS": "CHECK_ROOM_STATUS",
            "ROOM_STATUS": "CHECK_ROOM_STATUS",
            "MEETING_ROOM_STATUS": "CHECK_ROOM_STATUS",
        }
        return alias_map.get(upper, upper)

    def _normalize_fields(self, fields: Dict[str, Any]) -> Dict[str, Any]:
        """
        필드 키 표준화 및 허용되지 않은 키 제거
        """
        if not isinstance(fields, dict):
            return {}

        alias_map = {
            "destination_location": "dest_location",
            "destination": "dest_location",
            "dest": "dest_location",
            "from_location": "source_location",
            "pickup_location": "source_location",
            "query": "query_type",
        }

        allowed_fields = {
            "location",
            "item",
            "person_name",
            "source_location",
            "dest_location",
            "quantity",
            "device_type",
            "command",
            "target_value",
            "room_id",
            "meeting_room_id",
            "start_time",
            "end_time",
            "attendee_count",
            "area",
            "waypoints",
            "keywords",
            "message",
        }

        normalized: Dict[str, Any] = {}
        for key, value in fields.items():
            normalized_key = alias_map.get(key, key)
            if normalized_key in allowed_fields:
                normalized[normalized_key] = value

        return normalized

    def extract_entities(self, text: str) -> Dict[str, Any]:
        """
        텍스트에서 장소와 물품 정보 추출

        Args:
            text: 분석할 사용자 입력 텍스트

        Returns:
            추출된 엔티티 정보 {"location": str, "item": str, "confidence": float}
        """
        logger.info(f"엔티티 추출 요청: {text}")

        user_prompt = f"사용자 요청: {text}"

        try:
            result_text = self._chat(
                messages=[
                    {"role": "system", "content": ENTITY_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.1,
            )
            logger.info(f"LLM 응답: {result_text}")

            parsed = self._parse_json_response(result_text)

            # null을 None으로 변환
            location = self._normalize_null_value(parsed.get("location"))
            item = self._normalize_null_value(parsed.get("item"))

            result = {
                "location": location,
                "item": item,
                "confidence": 0.9,  # 기본 신뢰도
                "raw_text": self._sanitize_json_text(result_text),
            }

            logger.info(f"엔티티 추출 완료: location={location}, item={item}")
            return result

        except json.JSONDecodeError as e:
            logger.error(f"JSON 파싱 실패: {e}, 응답: {result_text}")
            return {
                "location": None,
                "item": None,
                "confidence": 0.0,
                "error": "JSON 파싱 실패",
                "raw_text": result_text,
            }
        except Exception as e:
            logger.error(f"엔티티 추출 실패: {e}")
            return {"location": None, "item": None, "confidence": 0.0, "error": str(e)}

    def parse_natural_language(self, text: str) -> Dict[str, Any]:
        """
        자연어 프롬프트를 구조화된 작업 메시지로 변환

        Args:
            text: 사용자의 자연어 프롬프트

        Returns:
            구조화된 작업 정보 딕셔너리
        """
        logger.info(f"자연어 프롬프트 해석 요청: {text[:100]}...")

        user_prompt = f"사용자 요청: {text}"

        try:
            result_text = self._chat(
                messages=[
                    {"role": "system", "content": PARSE_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=0.1,
            )
            logger.info(f"LLM 응답: {result_text[:200]}...")

            # JSON 파싱
            result_text = self._sanitize_json_text(result_text)
            parsed = self._parse_json_response(result_text)

            # task_type 검증
            task_type = self._normalize_task_type(parsed.get("task_type", "UNKNOWN"))
            confidence = parsed.get("confidence", 0.0)
            fields = self._normalize_fields(parsed.get("fields", {}))

            # null 값들을 None으로 변환
            for key, value in fields.items():
                fields[key] = self._normalize_null_value(value)

            result = {
                "task_type": task_type,
                "confidence": confidence,
                "fields": fields,
                "raw_text": result_text,
            }

            logger.info(
                f"자연어 해석 완료: task_type={task_type}, confidence={confidence}, fields={list(fields.keys())}"
            )
            return result

        except json.JSONDecodeError as e:
            logger.error(f"JSON 파싱 실패: {e}, 응답: {result_text}")
            return {
                "task_type": "UNKNOWN",
                "confidence": 0.0,
                "fields": {},
                "error": "JSON 파싱 실패",
                "raw_text": result_text,
            }
        except Exception as e:
            logger.error(f"자연어 해석 실패: {e}")
            return {
                "task_type": "UNKNOWN",
                "confidence": 0.0,
                "fields": {},
                "error": str(e),
            }
