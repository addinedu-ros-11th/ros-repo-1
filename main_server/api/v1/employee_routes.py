from fastapi import APIRouter, Depends, HTTPException, Cookie
from typing import Dict, Any
from main_server.container import container
from main_server.domains.tasks.schemas import ConfirmTaskRequest
import uuid
import logging

logger = logging.getLogger(__name__)

# ---------------------------------------------------------
# 1. 직원 권한 확인 및 라우터 설정
# ---------------------------------------------------------
async def verify_staff(user_role: str = Cookie(None)):
    """
    쿠키의 user_role을 검사하여 접속 권한을 확인합니다.
    """
    if user_role not in ["STAFF", "ADMIN"]:
        raise HTTPException(
            status_code=403, 
            detail="직원 권한이 없습니다."
        )

router = APIRouter(
    prefix="/api/v1/employee", 
    tags=["Employee"],
    dependencies=[Depends(verify_staff)] # 모든 엔드포인트에 권한 확인 적용
)

# ---------------------------------------------------------
# 2. AI Assistant 자연어 명령 처리 (Command)
# ---------------------------------------------------------
@router.post("/command")
async def process_command(request: Dict[str, Any]):
    """
    직원의 자연어 명령을 해석하여 작업을 생성하고 로봇을 할당합니다.
    """
    logger.info(f"Received command request: {request}")
    message = request.get("message")
    caller_id = request.get("user_id")

    if not message:
        raise HTTPException(status_code=400, detail="Message is required")
    
    if not caller_id:
        return {
            "status": "error", 
            "message": "사용자 식별 정보(user_id)가 없습니다. 로그인 후 다시 시도해주세요.",
            "received_body": request
        }

    req_id = str(uuid.uuid4())
    
    # AI 서비스를 통해 자연어 해석 실행
    ai_result = await container.ai_processing_service.process_natural_language(req_id, message)
    
    task_type = ai_result.get("task_type")
    
    # 1. 일반 대화/인사 처리 (로봇 작업 생성 안 함)
    if task_type in ["GREETING", "GENERAL_QUESTION"]:
        # AI 서버가 생성한 답변 텍스트를 그대로 반환
        answer_text = ai_result.get("fields", {}).get("message", "죄송합니다, 답변을 생성할 수 없습니다.")
        return {
            "status": "success", 
            "message": answer_text,
            "ai_fields": None 
        }

    if task_type == "UNKNOWN":
        return {"status": "error", "message": "명령을 이해하지 못했습니다.", "ai_result": ai_result}

    # 2. 로봇 작업 생성 (그 외 TaskType)
    task = await container.task_manager.create_task_from_ai(ai_result, caller_name=caller_id)
    
    if not task:
        return {"status": "retry", "message": "가용한 로봇이 없습니다.", "ai_result": ai_result}

    return {
        "status": "success",
        "message": f"작업이 접수되었습니다: {ai_result['task_type']}",
        "task_id": task.id,
        "ai_fields": ai_result.get("fields")
    }

# ---------------------------------------------------------
# 3. 작업 수행 확인 처리 (Confirm)
# ---------------------------------------------------------
@router.post("/confirm")
async def confirm_delivery_action(request: ConfirmTaskRequest):
    """
    로봇 도착 후 사용자의 수령 또는 적재 확인 액션을 처리합니다.
    """
    success, message = await container.task_manager.confirm_delivery(
        request.task_id, 
        request.action_type
    )
    
    if not success:
        return {"status": "error", "message": message}
        
    return {"status": "success", "message": message}

# ---------------------------------------------------------
# 4. 방문 예약 관리 API (Reservations)
# ---------------------------------------------------------

@router.get("/reservations/pending")
async def get_pending_reservations():
    """승인 대기 목록 조회"""
    return await container.reservation_repository.get_pending_list()

@router.get("/reservations/approved")
async def get_approved_reservations():
    """확정된 예약 현황 조회"""
    return await container.reservation_repository.get_approved_list()

@router.post("/reservations/apply")
async def apply_reservation(request: Dict[str, Any]):
    # 프론트에서 보낸 manager_name과 별도로 account 정보를 함께 저장
    res_id = await container.reservation_repository.create({
        "manager_account": request.get('manager_account'), # 추가됨
        "manager_name": request.get('manager_name'),
        "visitor_name": request['visitor_name'],
        "purpose": request['purpose'],
        "visit_date": request['visit_date'],
        "visit_time": request.get('visit_time'),
        "visitor_phone": request.get('visitor_phone'),
        "status": "PENDING"
    })
    return {"status": "success", "id": res_id}

@router.post("/reservations/decision")
async def decide_reservation(request: Dict[str, Any]):
    """승인(APPROVED) 또는 반려(REJECTED) 결정"""
    await container.reservation_repository.update_status(
        request['id'], 
        request['status']
    )
    return {"status": "success"}