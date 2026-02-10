"""
LLM gRPC Servicer Implementation
자연어 처리, 대화 생성, 의도 분석 기능 제공
"""

import logging
from typing import Any, Dict, Optional

import grpc

# ai_services.proto 기반 pb2 사용
from ai_server.grpc_impl import ai_services_pb2
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

    def ParseNaturalLanguage(self, request, context):
        """
        자연어 프롬프트를 구조화된 작업 메시지로 변환

        Args:
            request: NLRequest (req_id, message)
            context: gRPC context

        Returns:
            StructuredResponse (req_id, task_type, confidence, struct_msg)
        """
        logger.info(
            f"자연어 프롬프트 해석 요청 [req_id={request.req_id}]: {request.message[:50]}..."
        )

        try:
            # LLM 서비스를 통해 자연어 해석
            result = self.llm_service.parse_natural_language(request.message)

            # TaskType enum 매핑
            task_type_str = result.get("task_type", "UNKNOWN")
            task_type_enum = getattr(
                ai_services_pb2.TaskType,
                task_type_str,
                ai_services_pb2.TaskType.UNKNOWN,
            )

            # StructuredMessage 생성
            fields = result.get("fields", {})
            struct_msg_kwargs = self._build_structured_message_kwargs(fields)

            # StructuredMessage 생성
            struct_msg = ai_services_pb2.StructuredMessage(**struct_msg_kwargs)

            # StructuredResponse 생성
            response = ai_services_pb2.StructuredResponse(
                req_id=request.req_id,
                task_type=task_type_enum,
                confidence=result.get("confidence", 0.0),
                struct_msg=struct_msg,
                raw_text=result.get("raw_text", ""),
            )

            logger.info(
                f"자연어 해석 완료 [req_id={request.req_id}]: task_type={task_type_str}, confidence={result.get('confidence', 0.0)}"
            )
            return response

        except Exception as e:
            logger.error(f"자연어 해석 중 오류: {e}", exc_info=True)
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"자연어 해석 실패: {str(e)}")

            # 에러 시 기본 응답 반환
            return ai_services_pb2.StructuredResponse(
                req_id=request.req_id,
                task_type=ai_services_pb2.TaskType.UNKNOWN,
                confidence=0.0,
                struct_msg=ai_services_pb2.StructuredMessage(),
                raw_text=f"Error: {str(e)}",
            )

    def _build_structured_message_kwargs(
        self, fields: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        StructuredMessage 생성용 필드 변환
        """
        struct_msg_kwargs: Dict[str, Any] = {}

        def set_if_present(key: str, value: Any):
            if value is not None and value != "":
                struct_msg_kwargs[key] = value

        def coerce_int(key: str, value: Any):
            try:
                set_if_present(key, int(value))
            except (ValueError, TypeError):
                logger.warning(f"정수 변환 실패: {key}={value}")

        def coerce_float(key: str, value: Any):
            try:
                set_if_present(key, float(value))
            except (ValueError, TypeError):
                logger.warning(f"실수 변환 실패: {key}={value}")

        direct_fields = [
            "location",
            "item",
            "person_name",
            "person_id",
            "source_location",
            "dest_location",
            "room_id",
            "meeting_room_id",
            "start_time",
            "end_time",
            "area",
            "query_type",
            "message",
        ]

        for field_name in direct_fields:
            if field_name in fields:
                set_if_present(field_name, fields[field_name])

        if "quantity" in fields:
            coerce_int("quantity", fields["quantity"])
        if "attendee_count" in fields:
            coerce_int("attendee_count", fields["attendee_count"])
        if "target_value" in fields:
            coerce_float("target_value", fields["target_value"])

        if "device_type" in fields and fields["device_type"]:
            struct_msg_kwargs["device_type"] = getattr(
                ai_services_pb2.IoTDeviceType,
                fields["device_type"],
                ai_services_pb2.IoTDeviceType.IOT_UNKNOWN,
            )

        if "command" in fields and fields["command"]:
            struct_msg_kwargs["command"] = getattr(
                ai_services_pb2.IoTCommandType,
                fields["command"],
                ai_services_pb2.IoTCommandType.IOT_CMD_UNKNOWN,
            )

        if isinstance(fields.get("waypoints"), list):
            struct_msg_kwargs["waypoints"] = fields["waypoints"]
        if isinstance(fields.get("keywords"), list):
            struct_msg_kwargs["keywords"] = fields["keywords"]

        return struct_msg_kwargs
