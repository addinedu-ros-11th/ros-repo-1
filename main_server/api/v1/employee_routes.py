from fastapi import APIRouter, Depends, HTTPException, Cookie # Cookie 추가
from typing import Dict, Any
from main_server.container import container
from main_server.domains.tasks.schemas import ConfirmTaskRequest
import uuid

# 직원 확인용 함수
async def verify_staff(user_role: str = Cookie(None)):
    # 관리자(ADMIN)는 직원 페이지도 볼 수 있게 하려면 or user_role == "ADMIN" 추가
    if user_role not in ["STAFF", "ADMIN"]:
        raise HTTPException(
            status_code=403, 
            detail="직원 권한이 없습니다."
        )

# 라우터 전체에 문지기 적용
router = APIRouter(
    prefix="/api/v1/employee", 
    tags=["Employee"],
    dependencies=[Depends(verify_staff)] # 여기 추가!
)

import logging
logger = logging.getLogger(__name__)

@router.post("/command")
async def process_command(request: Dict[str, Any]):
    """
    직원의 자연어 명령을 처리합니다.
    Body 예시: {"message": "나한테 커피 가져다줘", "user_id": "worker123"}
    """
    logger.info(f"Received command request: {request}")
    message = request.get("message")
    caller_id = request.get("user_id")

    if not message:
        raise HTTPException(status_code=400, detail="Message is required")
    
    if not caller_id:
        # 앱에서 user_id를 보내지 않았을 경우
        return {
            "status": "error", 
            "message": "사용자 식별 정보(user_id)가 없습니다. 로그인 후 다시 시도해주세요.",
            "received_body": request
        }

    req_id = str(uuid.uuid4())
    
    # 1. AI 서비스를 통해 자연어 해석
    ai_result = await container.ai_processing_service.process_natural_language(req_id, message)
    
    if ai_result.get("task_type") == "UNKNOWN":
        return {"status": "error", "message": "명령을 이해하지 못했습니다.", "ai_result": ai_result}

    # 2. 해석된 데이터를 바탕으로 작업 생성 및 로봇 할당
    # caller_id를 전달하여 AI 결과에 requester_name이 없을 경우의 기본값으로 사용
    task = await container.task_manager.create_task_from_ai(ai_result, caller_name=caller_id)
    
    if not task:
        return {"status": "retry", "message": "가용한 로봇이 없습니다.", "ai_result": ai_result}

    return {
        "status": "success",
        "message": f"작업이 접수되었습니다: {ai_result['task_type']}",
        "task_id": task.id,
        "ai_fields": ai_result.get("fields")
    }

@router.post("/confirm")
async def confirm_delivery_action(request: ConfirmTaskRequest):
    """
    사용자의 수령/적재 확인을 처리합니다.
    """
    success, message = await container.task_manager.confirm_delivery(
        request.task_id, 
        request.action_type
    )
    
    if not success:
        return {"status": "error", "message": message}
        
    return {"status": "success", "message": message}
