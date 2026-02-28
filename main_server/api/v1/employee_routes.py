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
    
    # ---------------------------------------------------------
    # [TEST ONLY] go:(x,y,theta) 또는 go:(location_name) 수동 명령 가로채기
    # ---------------------------------------------------------
    if message.startswith("go:"):
        import re
        # 1. 좌표 직접 입력: go:(5,5,0)
        match_coord = re.match(r"go:\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", message.strip())
        
        # 2. 장소 이름 입력: go:(office_1)
        match_name = re.match(r"go:\(([^)]+)\)", message.strip())

        manual_result = None
        target_name = None

        if match_coord:
            x, y, theta = map(float, match_coord.groups())
            manual_result = {
                "task_type": "MANUAL_MOVE",
                "fields": {
                    "x": x, "y": y, "theta": theta,
                    "requester_name": caller_id
                }
            }
            target_name = f"({x}, {y})"

        elif match_name:
            loc_name = match_name.group(1).strip()
            # DB에서 장소 검색
            location_data = await container.location_repo.find_by_name(loc_name)
            
            if location_data:
                x = location_data.get("coordinate_x")
                y = location_data.get("coordinate_y")
                theta = location_data.get("theta", 0.0)
                
                manual_result = {
                    "task_type": "MANUAL_MOVE",
                    "fields": {
                        "x": x, "y": y, "theta": theta,
                        "requester_name": caller_id,
                        "destination_name": loc_name 
                    }
                }
                target_name = loc_name
            else:
                return {"status": "error", "message": f"위치 '{loc_name}'을(를) 찾을 수 없습니다."}

        if manual_result:
            task = await container.task_manager.create_task_from_ai(manual_result, caller_name=caller_id)
            if not task:
                return {"status": "retry", "message": "가용한 로봇이 없습니다."}
            
            return {
                "status": "success",
                "message": f"'{target_name}'(으)로 이동하고 있습니다.",
                "task_id": task.id,
                "ai_fields": manual_result["fields"]
            }

    # ---------------------------------------------------------
    # [TEST ONLY] cancle:(robot_id) 작업 취소 명령 가로채기
    # ---------------------------------------------------------
    elif message.lower().startswith("cancel:"):
        import re
        # cancle:(robot_1) or cancel:(1)
        match_cancel = re.match(r"cancell?e?:\(([^)]+)\)", message.strip(), re.IGNORECASE)
        
        if match_cancel:
            robot_identifier = match_cancel.group(1).strip()
            
            # 로봇 검색 (ID or Name)
            target_robot = None
            
            # Try as ID first if numeric
            if robot_identifier.isdigit():
                target_robot = await container.robot_repo.get_by_id(int(robot_identifier))
            
            # Try as Name if not found or not numeric
            if not target_robot:
                target_robot = await container.robot_repo.get_by_name(robot_identifier)
                
            if target_robot:
                # 1. 로봇에게 취소 명령 전송
                container.fleet_manager.cancel_robot_task(target_robot.name)
                
                # 2. DB 상의 Task 상태 업데이트 (CANCELLED)
                if target_robot.current_task_id:
                     task = await container.task_repo.get_by_id(target_robot.current_task_id)
                     if task and task.status not in ["COMPLETED", "CANCELLED", "FAILED"]:
                         await container.task_repo.update(task.id, {"status": "CANCELLED"})
                         
                return {
                    "status": "success", 
                    "message": f"로봇 '{target_robot.name}'의 작업을 취소했습니다."
                }
            else:
                return {
                    "status": "error", 
                    "message": f"로봇 '{robot_identifier}'을(를) 찾을 수 없습니다."
                }
    # ---------------------------------------------------------

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

    # 결과 메시지 및 필드 보정
    response_message = f"작업이 접수되었습니다: {task_type}"
    fields = ai_result.get("fields")
    if fields is None:
        fields = {}

    if task_type == "SNACK_DELIVERY":
        response_message = "간식 배달 요청이 접수되었습니다. 로봇이 탕비실에서 간식을 수령하여 요청하신 위치로 배달합니다."
        # 프론트엔드에 목적지가 명확히 나오도록 설정
        if not fields.get("dest_location") and not fields.get("location"):
             fields["dest_location"] = "요청자 위치"

    return {
        "status": "success",
        "message": response_message,
        "task_id": task.id,
        "ai_fields": fields
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