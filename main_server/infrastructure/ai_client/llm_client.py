import logging
import asyncio
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
        logger.info(f"LLM gRPC Client 초기화 완료 (Connecting to {host}:{port}).")
        # 연결 상태 모니터링 태스크 시작
        self._monitor_task = asyncio.create_task(self._monitor_connectivity())

    async def _monitor_connectivity(self):
        """gRPC 채널의 연결 상태를 모니터링하고 로그를 남깁니다."""
        last_state = None
        while True:
            state = self.channel.get_state(try_to_connect=True)
            if state != last_state:
                if state == grpc.ChannelConnectivity.READY:
                    logger.info(f"LLM gRPC 서버에 연결되었습니다. (State: {state})")
                elif state == grpc.ChannelConnectivity.TRANSIENT_FAILURE:
                    logger.warning(f"LLM gRPC 서버 연결 실패 - 재시도 중... (State: {state})")
                elif state == grpc.ChannelConnectivity.IDLE:
                    logger.info(f"LLM gRPC 서버 연결이 유휴 상태입니다. (State: {state})")
                elif state == grpc.ChannelConnectivity.CONNECTING:
                    logger.info(f"LLM gRPC 서버에 연결 시도 중... (State: {state})")
                
                last_state = state
            
            # 상태가 변경될 때까지 대기
            try:
                await self.channel.wait_for_state_change(last_state)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"LLM gRPC 상태 모니터링 오류: {e}")
                await asyncio.sleep(5)

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

        if struct_msg.HasField("requester_name"):
            fields["requester_name"] = struct_msg.requester_name
        if struct_msg.HasField("receiver_name"):
            fields["receiver_name"] = struct_msg.receiver_name
        if struct_msg.HasField("visitor_name"):
            fields["visitor_name"] = struct_msg.visitor_name

        if struct_msg.HasField("source_location"):
            fields["source_location"] = struct_msg.source_location
        if struct_msg.HasField("dest_location"):
            fields["dest_location"] = struct_msg.dest_location

        if len(struct_msg.items) > 0:
            fields["items"] = [
                {"item_name": item.item_name, "quantity": item.quantity}
                for item in struct_msg.items
            ]

        if struct_msg.HasField("message"):
            fields["message"] = struct_msg.message
        if len(struct_msg.keywords) > 0:
            fields["keywords"] = list(struct_msg.keywords)

        return fields

    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
