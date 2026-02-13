from fastapi import APIRouter, Depends, HTTPException
from typing import Dict, Any
from main_server.container import container
from main_server.domains.tasks.schemas import ConfirmTaskRequest
import uuid

router = APIRouter(prefix="/api/v1/employee", tags=["Employee"])

@router.post("/command")
async def process_command(request: Dict[str, str]):
    """
    직원의 자연어 명령(예: "커피 배달해줘")을 처리합니다.
    """
    message = request.get("message")
    if not message:
        raise HTTPException(status_code=400, detail="Message is required")

    req_id = str(uuid.uuid4())
    
    # 1. AI 서비스를 통해 자연어 해석
    ai_result = await container.ai_processing_service.process_natural_language(req_id, message)
    
    if ai_result.get("task_type") == "UNKNOWN":
        return {"status": "error", "message": "명령을 이해하지 못했습니다.", "ai_result": ai_result}

    # 2. 해석된 데이터를 바탕으로 작업 생성 및 로봇 할당
    task = await container.task_manager.create_task_from_ai(ai_result)
    
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
