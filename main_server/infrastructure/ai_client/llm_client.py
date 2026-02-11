import logging
from typing import Any, Dict

import grpc
from main_server.config import config

# Generated gRPC files
from main_server.infrastructure.grpc import ai_llm_pb2
from main_server.infrastructure.grpc import ai_llm_pb2_grpc

logger = logging.getLogger(__name__)


class LLMServiceClient:
    """
    gRPC를 통해 원격 LLM 서버(Qwen3-4B)와 통신하는 클라이언트 서비스.
    """

    def __init__(
        self, host: str = config.LLM_GRPC_HOST, port: int = config.LLM_GRPC_PORT
    ):
        self.channel = grpc.aio.insecure_channel(f"{host}:{port}")
        self.stub = ai_llm_pb2_grpc.LLMServiceStub(self.channel)
        print(f"LLM gRPC Client 초기화 완료 (Connecting to {host}:{port}).")

    async def parse_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
        """
        자연어 프롬프트를 해석하여 구조화된 작업 데이터로 변환합니다.
        """
        request = ai_llm_pb2.NLRequest(req_id=req_id, message=message)
        response = await self.stub.ParseNaturalLanguage(request)

        return {
            "req_id": response.req_id,
            "task_type": ai_llm_pb2.TaskType.Name(response.task_type),
            "confidence": response.confidence,
            "fields": self._struct_msg_to_dict(response.struct_msg),
            "raw_text": response.raw_text,
        }

    def _struct_msg_to_dict(
        self, struct_msg: ai_llm_pb2.StructuredMessage
    ) -> Dict[str, Any]:
        """
        StructuredMessage를 일반 dict로 변환합니다.
        """
        fields: Dict[str, Any] = {}

        if struct_msg.HasField("location"):
            fields["location"] = struct_msg.location
        if struct_msg.HasField("item"):
            fields["item"] = struct_msg.item
        if struct_msg.HasField("person_name"):
            fields["person_name"] = struct_msg.person_name
        if struct_msg.HasField("person_id"):
            fields["person_id"] = struct_msg.person_id

        if struct_msg.HasField("source_location"):
            fields["source_location"] = struct_msg.source_location
        if struct_msg.HasField("dest_location"):
            fields["dest_location"] = struct_msg.dest_location
        if struct_msg.HasField("quantity"):
            fields["quantity"] = struct_msg.quantity

        if struct_msg.HasField("device_type"):
            fields["device_type"] = ai_llm_pb2.IoTDeviceType.Name(
                struct_msg.device_type
            )
        if struct_msg.HasField("command"):
            fields["command"] = ai_llm_pb2.IoTCommandType.Name(struct_msg.command)
        if struct_msg.HasField("target_value"):
            fields["target_value"] = struct_msg.target_value
        if struct_msg.HasField("room_id"):
            fields["room_id"] = struct_msg.room_id

        if struct_msg.HasField("meeting_room_id"):
            fields["meeting_room_id"] = struct_msg.meeting_room_id
        if struct_msg.HasField("start_time"):
            fields["start_time"] = struct_msg.start_time
        if struct_msg.HasField("end_time"):
            fields["end_time"] = struct_msg.end_time
        if struct_msg.HasField("attendee_count"):
            fields["attendee_count"] = struct_msg.attendee_count

        if struct_msg.HasField("area"):
            fields["area"] = struct_msg.area
        if len(struct_msg.waypoints) > 0:
            fields["waypoints"] = list(struct_msg.waypoints)

        if struct_msg.HasField("query_type"):
            fields["query_type"] = struct_msg.query_type
        if struct_msg.HasField("message"):
            fields["message"] = struct_msg.message
        if len(struct_msg.keywords) > 0:
            fields["keywords"] = list(struct_msg.keywords)

        return fields

    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
