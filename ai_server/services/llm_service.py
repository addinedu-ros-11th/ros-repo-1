"""
LLM Service - Qwen3-4B with Ollama
LLM 관련 기능을 처리하는 서비스 레이어
"""

import logging
import json
import re
from typing import Dict, Any, List
import ollama

logger = logging.getLogger(__name__)


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

    def chat(self, messages: list) -> str:
        """
        대화형 응답 생성

        Args:
            messages: 대화 히스토리 [{"role": "user"/"assistant", "content": "..."}]

        Returns:
            생성된 응답
        """
        logger.info(f"대화 요청: {len(messages)} 메시지")

        try:
            response = ollama.chat(model=self.model_name, messages=messages)
            result = response["message"]["content"]
            logger.info(f"대화 응답 생성 완료")
            return result
        except Exception as e:
            logger.error(f"대화 처리 실패: {e}")
            raise

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

    def extract_entities(self, text: str) -> Dict[str, Any]:
        """
        텍스트에서 장소와 물품 정보 추출

        Args:
            text: 분석할 사용자 입력 텍스트

        Returns:
            추출된 엔티티 정보 {"location": str, "item": str, "confidence": float}
        """
        logger.info(f"엔티티 추출 요청: {text}")

        # 프롬프트 엔지니어링: 구조화된 출력
        system_prompt = """당신은 사무실 로봇을 위한 명령어 파서입니다.
사용자의 요청에서 다음 정보만 추출하세요:
1. location (장소): 예) 회의실, 로비, 사무실, 301호, A동 등
2. item (물품): 예) 커피, 서류, 노트북, 상자 등

반드시 다음 JSON 형식으로만 답변하세요:
{"location": "장소이름 또는 null", "item": "물품이름 또는 null"}

중요:
- 장소나 물품이 없으면 null로 표시
- JSON 형식 외의 다른 텍스트는 절대 포함하지 마세요
- 추가 설명이나 인사말도 하지 마세요"""

        user_prompt = f"사용자 요청: {text}"

        try:
            response = ollama.chat(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                options={"temperature": 0.1},  # 낮은 온도로 일관된 응답
            )

            result_text = response["message"]["content"].strip()
            logger.info(f"LLM 응답: {result_text}")

            # JSON 파싱
            # 때때로 모델이 마크다운 코드 블록으로 감싸는 경우 제거
            result_text = re.sub(r"```json\s*|\s*```", "", result_text)
            result_text = result_text.strip()

            parsed = json.loads(result_text)

            # null을 None으로 변환
            location = parsed.get("location")
            item = parsed.get("item")

            if location == "null":
                location = None
            if item == "null":
                item = None

            result = {
                "location": location,
                "item": item,
                "confidence": 0.9,  # 기본 신뢰도
                "raw_text": result_text,
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
