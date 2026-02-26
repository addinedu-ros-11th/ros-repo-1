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
2. items (물품 목록): 예) [{"item_name": "커피", "quantity": 2}]
3. requester_name (요청자 이름): 예) 홍길동
4. receiver_name (수신자 이름): 예) 김철수

반드시 다음 JSON 형식으로만 답변하세요:
{"location": "장소이름 또는 null", "items": [{"item_name": "물품명", "quantity": 1}] 또는 null, "requester_name": "이름 또는 null", "receiver_name": "이름 또는 null"}

중요:
- 해당 없는 필드는 반드시 null로 표시
- JSON 형식 외의 다른 텍스트는 절대 포함하지 마세요
- 추가 설명이나 인사말도 하지 마세요

[절대 금지 - 정보 조작 방지]
- 사용자가 입력에서 명시적으로 언급하지 않은 정보는 절대 추측하거나 만들어내지 마세요
- 특히 requester_name, receiver_name은 사용자 입력에 실제 이름(예: 홍길동, 김철수)이 있을 때만 추출하세요
- "사용자", "본인", "요청자", "나", "고객" 같은 일반적/대명사적 표현은 이름이 아닙니다. 이런 경우 반드시 null로 처리하세요
- location도 사용자가 구체적 장소를 명시한 경우에만 추출하세요. "사무실", "현재 위치" 같은 추측은 금지합니다
- 입력에 없는 정보를 추가하면 시스템 오류가 발생합니다. 없으면 반드시 null입니다"""

PARSE_SYSTEM_PROMPT = """당신은 사무실 로봇 서비스를 위한 자연어 명령 파서입니다.
사용자의 요청을 분석하여 작업 유형과 관련 정보를 추출하세요.

작업 유형 (task_type):
1. SNACK_DELIVERY: 간식 배달 (예: "커피 갖다줘", "간식 가져와줘")
2. ITEM_DELIVERY: P2P 물품 배달 (예: "김철수한테 서류 전달해줘")
3. GUIDE_GUEST: 방문객 안내 (예: "손님을 회의실로 안내해줘")
4. GENERAL_QUESTION: 일반 질문 (예: "오늘 날씨 어때?", "회사 규정 알려줘")
5. GREETING: 인사 (예: "안녕", "반가워")

추출할 필드:
- location: 위치/장소 (일반적 의미)
- requester_name: 요청자 이름 (User DB 매칭용)
- receiver_name: 수신자 이름 (ITEM_DELIVERY 시)
- visitor_name: 방문객 이름 (GUIDE_GUEST 시)
- source_location: 출발지 (명시적 지정 시)
- dest_location: 목적지 (명시적 지정 시)
- items: 물품 목록 (배열, 각 항목은 {"item_name": "물품명", "quantity": 수량})
- keywords: 키워드 목록 (배열)
- message: 부가 메시지

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
    location, requester_name, receiver_name, visitor_name,
    source_location, dest_location, items,
    keywords, message
- items는 [{"item_name": "커피", "quantity": 2}] 형식의 배열로
- JSON 외의 다른 텍스트는 절대 포함하지 마세요
- 배열 필드는 ["item1", "item2"] 형식으로

[절대 금지 - 정보 조작 방지]
- 사용자 입력에 명시적으로 언급되지 않은 정보를 절대 추측하거나 만들어내지 마세요
- 입력에 없는 정보를 임의로 채우면 시스템 오류가 발생합니다
- requester_name: 실제 사람 이름(예: 홍길동, 김철수)이 입력에 있을 때만 추출. "사용자", "본인", "요청자", "나", "고객" 등 일반 표현은 이름이 아님 → fields에 포함하지 마세요
- receiver_name, visitor_name: 실제 사람 이름이 입력에 있을 때만 추출. 일반 표현은 포함하지 마세요
- location, source_location, dest_location: 구체적 장소명이 입력에 있을 때만 추출. 추측 금지
- 해당하지 않는 필드는 fields 객체에 아예 포함하지 마세요 (null도 넣지 마세요)

예시:
입력: "초코파이 갖다줘"
출력: {"task_type": "SNACK_DELIVERY", "confidence": 0.95, "fields": {"items": [{"item_name": "초코파이", "quantity": 1}]}}
(requester_name, location 등 언급되지 않은 필드는 포함하지 않음)

입력: "홍길동인데 회의실에 커피 2잔 갖다줘"
출력: {"task_type": "SNACK_DELIVERY", "confidence": 0.95, "fields": {"requester_name": "홍길동", "location": "회의실", "items": [{"item_name": "커피", "quantity": 2}]}}"""


# ── 의도 분류 프롬프트 (Stage 1) ──────────────────────────
INTENT_CLASSIFY_PROMPT = """당신은 사무실 로봇 서비스의 의도 분류기입니다.
사용자의 입력이 다음 중 어느 카테고리에 해당하는지 판단하세요:

1. COMMAND: 로봇에게 작업을 요청하는 명령 (배달, 안내 등)
   예: "커피 갖다줘", "김철수한테 서류 전달해줘", "손님을 회의실로 안내해줘"

2. GREETING: 인사 표현
   예: "안녕", "반가워", "좋은 아침", "수고해"

3. GENERAL_QUESTION: 일반 질문이나 대화
   예: "오늘 날씨 어때?", "회사 규정 알려줘", "고마워", "뭐 할 수 있어?"

반드시 COMMAND, GREETING, GENERAL_QUESTION 중 하나만 답변하세요.
다른 텍스트는 절대 포함하지 마세요."""

# ── 일반 챗 응답 프롬프트 ──────────────────────────────────
CHAT_RESPONSE_PROMPT = """당신은 친절한 사무실 로봇 도우미입니다.
이름은 '오피스봇'이고, 사무실에서 간식 배달, 물품 전달, 방문객 안내 등을 수행합니다.
사용자와 자연스럽고 간결하게 대화하세요.
- 인사에는 밝고 짧게 답하세요.
- 일반 질문에는 도움이 되는 정보를 간결하게 제공하세요.
- 너무 길지 않게, 2~3문장 이내로 답변하세요."""


class LLMService:
    """
    LLM(Qwen3-4B) 모델을 사용한 자연어 처리 서비스
    Ollama를 통해 로컬 모델 실행
    """

    # 일반 챗 유형 (명령이 아닌 대화)
    CHAT_TASK_TYPES = {"GREETING", "GENERAL_QUESTION"}

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

    # LLM이 만들어내는 일반적/대명사적 placeholder 표현 목록
    _FABRICATED_NAME_PATTERNS = {
        "사용자",
        "본인",
        "요청자",
        "나",
        "고객",
        "직원",
        "회원",
        "수신자",
        "받는사람",
        "상대방",
        "방문객",
        "손님",
        "게스트",
        "user",
        "requester",
        "receiver",
        "visitor",
        "guest",
        "me",
        "self",
        "unknown",
        "n/a",
        "none",
        "미확인",
        "불명",
        "익명",
        "누군가",
        "아무개",
        "담당자",
    }

    _FABRICATED_LOCATION_PATTERNS = {
        "현재 위치",
        "현재위치",
        "여기",
        "이곳",
        "사무실",
        "미정",
        "미지정",
        "해당없음",
        "해당 없음",
        "unknown",
        "n/a",
        "none",
        "here",
        "미확인",
    }

    def _normalize_null_value(self, value: Any) -> Any:
        """
        null/빈 문자열/자연어 "없음" 표현을 None으로 표준화
        """
        if value is None:
            return None
        if isinstance(value, str):
            lower_val = value.strip().lower()
            # null 패턴 감지
            if lower_val in {"null", ""}:
                return None
            # 자연어 "없음" 패턴 감지 (LLM이 JSON 형식을 따르지 않을 때 대비)
            if any(
                keyword in lower_val
                for keyword in [
                    "없음",
                    "없이",
                    "미명시",
                    "명시되지",
                    "확인불가",
                    "알 수 없",
                    "지정되지",
                    "언급되지",
                    "파악불가",
                ]
            ):
                return None
        return value

    def _is_fabricated_name(self, value: Any) -> bool:
        """
        LLM이 만들어낸 일반적/대명사적 이름인지 확인
        실제 사람 이름이 아닌 placeholder 표현을 감지
        """
        if value is None:
            return True
        if not isinstance(value, str):
            return False
        stripped = value.strip().lower()
        return stripped in self._FABRICATED_NAME_PATTERNS

    def _is_fabricated_location(self, value: Any) -> bool:
        """
        LLM이 만들어낸 일반적 장소 표현인지 확인
        구체적 장소가 아닌 placeholder 표현을 감지
        """
        if value is None:
            return True
        if not isinstance(value, str):
            return False
        stripped = value.strip().lower()
        return stripped in self._FABRICATED_LOCATION_PATTERNS

    def _sanitize_fabricated_fields(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        LLM 응답에서 조작된(fabricated) 필드 값을 None으로 변환
        입력에 없는 정보를 LLM이 임의로 채운 경우를 방지
        """
        # 이름 필드 검증
        name_fields = ["requester_name", "receiver_name", "visitor_name"]
        for field in name_fields:
            if field in data and self._is_fabricated_name(data[field]):
                logger.warning(
                    f"Fabricated 값 감지 → {field}='{data[field]}' → None으로 변환"
                )
                data[field] = None

        # 장소 필드 검증
        location_fields = ["location", "source_location", "dest_location"]
        for field in location_fields:
            if field in data and self._is_fabricated_location(data[field]):
                logger.warning(
                    f"Fabricated 값 감지 → {field}='{data[field]}' → None으로 변환"
                )
                data[field] = None

        return data

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
        alias_map = {}
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
            "requester": "requester_name",
            "receiver": "receiver_name",
            "visitor": "visitor_name",
            "guest_name": "visitor_name",
        }

        allowed_fields = {
            "location",
            "requester_name",
            "receiver_name",
            "visitor_name",
            "source_location",
            "dest_location",
            "items",
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
            추출된 엔티티 정보 {"location": str, "items": list, "confidence": float}
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
            items = parsed.get("items")
            if items and not isinstance(items, list):
                items = None
            requester_name = self._normalize_null_value(parsed.get("requester_name"))
            receiver_name = self._normalize_null_value(parsed.get("receiver_name"))

            result = {
                "location": location,
                "items": items,
                "requester_name": requester_name,
                "receiver_name": receiver_name,
                "confidence": 0.9,  # 기본 신뢰도
                "raw_text": self._sanitize_json_text(result_text),
            }

            # LLM이 임의로 만들어낸 값(fabricated) 제거
            result = self._sanitize_fabricated_fields(result)

            logger.info(f"엔티티 추출 완료: location={location}, items={items}")
            return result

        except json.JSONDecodeError as e:
            logger.error(f"JSON 파싱 실패: {e}, 응답: {result_text}")
            return {
                "location": None,
                "items": None,
                "confidence": 0.0,
                "error": "JSON 파싱 실패",
                "raw_text": result_text,
            }
        except Exception as e:
            logger.error(f"엔티티 추출 실패: {e}")
            return {"location": None, "items": None, "confidence": 0.0, "error": str(e)}

    # ------------------------------------------------------------------ #
    #  Stage 1: 빠른 의도 분류 (COMMAND / GREETING / GENERAL_QUESTION)
    # ------------------------------------------------------------------ #
    def _classify_intent(self, text: str) -> str:
        """
        사용자 입력의 의도를 빠르게 분류한다.

        Returns:
            "COMMAND" | "GREETING" | "GENERAL_QUESTION"
        """
        try:
            raw = self._chat(
                messages=[
                    {"role": "system", "content": INTENT_CLASSIFY_PROMPT},
                    {"role": "user", "content": text},
                ],
                temperature=0.0,
            )
            intent = raw.strip().upper()
            # 유효 값인지 확인
            if intent in ("COMMAND", "GREETING", "GENERAL_QUESTION"):
                logger.info(f"의도 분류 결과: {intent}")
                return intent
            # LLM이 부가 텍스트를 포함했을 경우 패턴 매칭
            for label in ("COMMAND", "GREETING", "GENERAL_QUESTION"):
                if label in intent:
                    logger.info(f"의도 분류 결과(패턴): {label}")
                    return label
            logger.warning(f"의도 분류 실패, 기본값 COMMAND 사용: raw={raw}")
            return "COMMAND"
        except Exception as e:
            logger.error(f"의도 분류 중 오류, 기본값 COMMAND: {e}")
            return "COMMAND"

    # ------------------------------------------------------------------ #
    #  일반 챗(Greeting / General Question) 응답 생성
    # ------------------------------------------------------------------ #
    def _generate_chat_response(self, text: str, task_type: str) -> Dict[str, Any]:
        """
        인사 또는 일반 질문에 대해 단순 대화 응답을 생성한다.
        명령 파싱 없이 자연어 응답만 반환.
        """
        logger.info(f"[Chat] 일반 챗 응답 생성 (type={task_type}): {text[:80]}")
        try:
            chat_reply = self._chat(
                messages=[
                    {"role": "system", "content": CHAT_RESPONSE_PROMPT},
                    {"role": "user", "content": text},
                ],
                temperature=0.7,
            )
            logger.info(f"[Chat] 응답 생성 완료: {chat_reply[:100]}")
        except Exception as e:
            logger.error(f"[Chat] 응답 생성 실패: {e}")
            chat_reply = "죄송합니다, 응답을 생성하지 못했습니다."

        return {
            "task_type": task_type,
            "confidence": 1.0,
            "fields": {
                "message": chat_reply,
            },
            "raw_text": chat_reply,
            "is_chat": True,
        }

    # ------------------------------------------------------------------ #
    #  명령(Command) 구조화 파싱 - 기존 로직
    # ------------------------------------------------------------------ #
    def _parse_command(self, text: str) -> Dict[str, Any]:
        """
        명령(SNACK_DELIVERY, ITEM_DELIVERY, GUIDE_GUEST 등)을
        구조화된 작업 메시지로 변환한다.
        """
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

            # LLM이 임의로 만들어낸 값(fabricated) 제거
            fields = self._sanitize_fabricated_fields(fields)

            # None 값인 필드는 fields에서 제거 (불필요한 빈 필드 정리)
            fields = {k: v for k, v in fields.items() if v is not None}

            result = {
                "task_type": task_type,
                "confidence": confidence,
                "fields": fields,
                "raw_text": result_text,
                "is_chat": False,
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
                "is_chat": False,
            }
        except Exception as e:
            logger.error(f"자연어 해석 실패: {e}")
            return {
                "task_type": "UNKNOWN",
                "confidence": 0.0,
                "fields": {},
                "error": str(e),
                "is_chat": False,
            }

    # ------------------------------------------------------------------ #
    #  공개 API: parse_natural_language  (2-Stage)
    # ------------------------------------------------------------------ #
    def parse_natural_language(self, text: str) -> Dict[str, Any]:
        """
        자연어 프롬프트를 구조화된 작업 메시지로 변환

        Stage 1 - 의도 분류 (COMMAND / GREETING / GENERAL_QUESTION)
        Stage 2a - 일반 챗이면 → 단순 대화 응답 생성
        Stage 2b - 명령이면 → 구조화된 파싱

        Args:
            text: 사용자의 자연어 프롬프트

        Returns:
            구조화된 작업 정보 딕셔너리
        """
        logger.info(f"자연어 프롬프트 해석 요청: {text[:100]}...")

        # Stage 1: 빠른 의도 분류
        intent = self._classify_intent(text)

        # Stage 2: 분기 처리
        if intent in self.CHAT_TASK_TYPES:
            # 일반 챗 (GREETING / GENERAL_QUESTION) → 대화 응답만 생성
            return self._generate_chat_response(text, task_type=intent)
        else:
            # 명령 → 구조화 파싱
            return self._parse_command(text)
