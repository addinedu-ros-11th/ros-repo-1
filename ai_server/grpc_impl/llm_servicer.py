"""
LLM gRPC Servicer Implementation
자연어 처리, 대화 생성, 의도 분석 기능 제공
"""

import logging
from typing import Optional
import grpc

# ai_services.proto 기반 pb2 사용
from ai_server.grpc_impl import ai_services_pb2
from ai_server.grpc_impl import ai_services_pb2_grpc
from ai_server.services.llm_service import LLMService

logger = logging.getLogger(__name__)


class LLMServicer:
    """
    LLM 전용 gRPC Servicer

    TODO: ai_services.proto 기반으로 생성된 LLMServiceServicer 사용
    현재는 기존 구조를 유지하면서 LLM 기능만 제공
    """

    def __init__(self, llm_service: Optional[LLMService] = None):
        """
        LLM Servicer 초기화

        Args:
            llm_service: LLM 서비스 인스턴스
        """
        self.llm_service = llm_service
        logger.info("LLM Servicer 초기화 완료")

    def GenerateText(self, request, context):
        """
        텍스트 생성 요청 처리

        Args:
            request: TextRequest
            context: gRPC context

        Returns:
            TextResponse
        """
        logger.info(f"텍스트 생성 요청: {request.text[:50]}...")

        try:
            # LLM 서비스를 통해 텍스트 생성
            result = self.llm_service.generate_text(
                prompt=request.text,
                max_length=request.max_length if request.max_length else 100,
            )

            # TODO: ai_services_pb2.TextResponse로 교체
            logger.info(f"텍스트 생성 완료: {len(result)} 문자")
            return {"generated_text": result, "confidence": 0.95}

        except Exception as e:
            logger.error(f"텍스트 생성 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"텍스트 생성 실패: {str(e)}")
            raise

    def Chat(self, request, context):
        """
        대화형 응답 생성

        Args:
            request: ChatRequest
            context: gRPC context

        Returns:
            ChatResponse
        """
        logger.info(f"대화 요청: {len(request.messages)} 메시지")

        try:
            # 메시지 변환
            messages = [
                {"role": msg.role, "content": msg.content} for msg in request.messages
            ]

            # LLM 서비스를 통해 대화 응답 생성
            result = self.llm_service.chat(messages)

            logger.info(f"대화 응답 생성 완료")
            return {"response": result, "confidence": 0.92}

        except Exception as e:
            logger.error(f"대화 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"대화 실패: {str(e)}")
            raise

    def AnalyzeIntent(self, request, context):
        """
        의도 분석 및 엔티티 추출 (장소, 물품)

        Args:
            request: TextRequest
            context: gRPC context

        Returns:
            IntentResponse (intent, confidence, entities)
        """
        logger.info(f"의도 분석 및 엔티티 추출 요청: {request.text[:50]}...")

        try:
            # LLM 서비스를 통해 엔티티 추출
            result = self.llm_service.extract_entities(request.text)

            # IntentResponse 형식으로 변환
            entities = []

            # 장소 엔티티 추가
            if result.get("location"):
                entities.append(
                    ai_services_pb2.Entity(
                        type="location",
                        value=result["location"],
                        confidence=result.get("confidence", 0.9),
                    )
                )

            # 물품 엔티티 추가
            if result.get("item"):
                entities.append(
                    ai_services_pb2.Entity(
                        type="item",
                        value=result["item"],
                        confidence=result.get("confidence", 0.9),
                    )
                )

            # Intent 결정
            if result.get("location") and result.get("item"):
                intent = "deliver_item_to_location"
            elif result.get("location"):
                intent = "navigate_to_location"
            elif result.get("item"):
                intent = "find_item"
            else:
                intent = "unknown"

            response = ai_services_pb2.IntentResponse(
                intent=intent,
                confidence=result.get("confidence", 0.9),
                entities=entities,
            )

            logger.info(f"엔티티 추출 완료: intent={intent}, entities={len(entities)}")
            return response

        except Exception as e:
            logger.error(f"의도 분석 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"의도 분석 실패: {str(e)}")
            raise
