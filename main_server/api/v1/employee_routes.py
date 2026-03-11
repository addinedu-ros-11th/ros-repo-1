from fastapi import APIRouter, Depends, HTTPException, Cookie
from typing import Dict, Any, Optional
from main_server.container import container
from main_server.domains.tasks.schemas import ConfirmTaskRequest, TaskStatus
from datetime import date, time, datetime
from pydantic import BaseModel
import uuid
import logging

logger = logging.getLogger(__name__)

class UserModel(BaseModel):
    user_id: int
    account: str
    name: Optional[str] = None
    department: Optional[str] = None
    role: Optional[str] = None
    location_id: Optional[int] = None
    created_at: Optional[datetime] = None # 이제 AttributeError가 나지 않습니다.

class RoomReservationRequest(BaseModel):
    user_id: str  # 또는 int, 프론트에서 넘어오는 데이터 타입에 맞춤
    location_id: int
    reservation_date: date
    start_time: str # "HH:mm" 형식
    end_time: str   # "HH:mm" 형식

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
    # [TEST ONLY] go:(...) 및 QR_SCAN:(...) 명령 처리
    # ---------------------------------------------------------
    
    # 1. 로그인한 유저 정보 조회 (Task 생성을 위해 필요)
    user_info = None
    if caller_id:
        user_info = await container.user_repo.get_user_by_username(caller_id)
    
    requester_pk = user_info.user_id if user_info else None

    # [CASE A] go: 명령어 처리
    if message.startswith("go:"):
        import re
        
        # 패턴 0: 특정 로봇 지정 이동 -> go:(robot_1, 3)
        match_specific = re.match(r"go:\(([^,]+),\s*(\d+)\)", message.strip())
        
        # 패턴 1: 좌표 직접 입력 -> go:(5.0, 5.0, 0.0)
        match_coord = re.match(r"go:\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)", message.strip())
        
        # 패턴 2: 장소 이름 입력 (자동 배차) -> go:(office_1)
        match_name = re.match(r"go:\(([^)]+)\)", message.strip())

        if match_specific:
            # [수동 배차] 특정 로봇을 지정된 위치 ID로 이동
            robot_name = match_specific.group(1).strip()
            location_id = int(match_specific.group(2).strip())
            
            robot = await container.robot_repo.get_by_name(robot_name)
            if not robot:
                return {"status": "error", "message": f"로봇 '{robot_name}'을(를) 찾을 수 없습니다."}
            
            loc = await container.location_repo.find_by_id(location_id)
            if not loc:
                return {"status": "error", "message": f"위치 ID {location_id}를 찾을 수 없습니다."}

            task_data = {
                "requester_id": requester_pk,
                "task_type": "MANUAL_MOVE",
                "priority": 1,
                "status": "ASSIGNED",
                "assigned_robot_id": robot.id,
                "destination_id": loc['location_id'],
                "target_location_name": loc['name'],
                "details": {
                    "x": loc["coordinate_x"],
                    "y": loc["coordinate_y"],
                    "theta": loc.get("theta", 0.0),
                    "requester_name": caller_id
                }
            }
            
            # 직접 Task 생성 및 할당
            task = await container.task_repo.create(task_data)
            if task:
                await container.task_manager.assign_and_dispatch(robot, task)
                return {
                    "status": "success",
                    "message": f"'{robot_name}'이(가) '{loc['name']}'(으)로 이동합니다.",
                    "robot_info": {
                        "robot_id": robot.id,
                        "name": robot.name,
                        "status": robot.status,
                        "battery": robot.battery_level
                    }
                }
            else:
                return {"status": "error", "message": "태스크 생성 실패"}

        # [자동 배차] 기존 로직 유지 (MANUAL_MOVE Task 생성 후 TaskManager에 위임)
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
                return {"status": "error", "message": "태스크 생성 실패"}
            
            msg = f"'{target_name}'(으)로 이동하고 있습니다."
            if task.status == TaskStatus.PENDING:
                msg = f"'{target_name}'(으)로 이동하기 위해 대기 중입니다."

            return {
                "status": "success",
                "message": msg,
                "task_id": task.id,
                "ai_fields": manual_result["fields"]
            }

    # [CASE B] QR_SCAN 명령어 처리
    elif message.startswith("QR_SCAN:"):
        import re
        match_qr = re.match(r"QR_SCAN:\(([^)]+)\)", message.strip())
        if match_qr:
            robot_name = match_qr.group(1).strip()
            robot = await container.robot_repo.get_by_name(robot_name)
            if not robot:
                 return {"status": "error", "message": f"로봇 '{robot_name}'을(를) 찾을 수 없습니다."}
            
            # GUEST_CHECK 태스크를 생성하여 프로세서가 응답을 처리하게 함
            task_data = {
                "requester_id": requester_pk,
                "task_type": "GUEST_CHECK",
                "priority": 1,
                "status": "ASSIGNED",
                "assigned_robot_id": robot.id,
                "details": {"purpose": "VISITOR_SCAN", "reason": "manual_test"}
            }
            
            task = await container.task_repo.create(task_data)
            if task:
                await container.task_manager.assign_and_dispatch(robot, task)
                return {
                    "status": "success",
                    "message": f"'{robot_name}'에게 방문객 QR 스캔 태스크를 할당했습니다.",
                    "robot_info": {
                        "robot_id": robot.id,
                        "name": robot.name,
                        "status": robot.status,
                        "battery": robot.battery_level
                    }
                }
            else:
                return {"status": "error", "message": "태스크 생성 실패"}

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
        # DB 저장 실패 등의 심각한 오류
        return {"status": "error", "message": "태스크 생성 중 오류가 발생했습니다.", "ai_result": ai_result}

    # 결과 메시지 및 필드 보정
    type_kor = {
        "SNACK_DELIVERY": "간식 배달",
        "ITEM_DELIVERY": "물품 배달",
        "MANUAL_MOVE": "장소 이동",
        "GUEST_GUIDE": "방문객 안내"
    }.get(task_type, task_type)

    response_message = f"[{type_kor}] 작업이 접수되었습니다."
    
    if task.status == TaskStatus.PENDING:
        response_message = f"현재 모든 로봇이 업무 중입니다. [{type_kor}] 작업을 대기열에 등록했습니다."

    fields = ai_result.get("fields", {})

    if task_type == "SNACK_DELIVERY":
        if task.status == TaskStatus.PENDING:
             response_message = "간식 배달 요청이 대기 중입니다. 로봇이 배정되는 대로 시작됩니다."
        else:
             response_message = "간식 배달을 위해 로봇이 탕비실로 이동하고 있습니다."
        
        if not fields.get("dest_location") and not fields.get("location"):
             fields["dest_location"] = "요청자 위치"

    # 로봇 정보 및 추가 메시지 구성
    robot_info = None
    if task.assigned_robot_id:
        robot = await container.robot_repo.get_by_id(task.assigned_robot_id)
        if robot:
            robot_info = {
                "robot_id": robot.id,
                "name": robot.name,
                "status": robot.status,
                "battery": robot.battery_level
            }
            if task.status == TaskStatus.ASSIGNED:
                response_message += f"\n알림: '{robot.name}' 로봇이 배차되었습니다. (배터리: {robot.battery_level}%)"

    return {
        "status": "success",
        "message": response_message,
        "task_id": task.id,
        "ai_fields": fields,
        "robot_info": robot_info
    }

# ---------------------------------------------------------
# 3. 작업 수행 확인 처리 (Confirm)
# ---------------------------------------------------------

@router.get("/tasks/my")
async def get_my_tasks(user_id: str = Cookie(None)):
    if not user_id:
        raise HTTPException(status_code=401, detail="인증 정보가 없습니다.")

    user = await container.user_repo.get_user_by_username(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    tasks = await container.task_repo.get_all_for_user(user.user_id)
    
    return [
        {
            "task_id": t.id,
            "task_type": t.task_type,
            "status": t.status,
            "created_at": t.created_at.strftime("%Y-%m-%d %H:%M") if t.created_at else "-",
            # 이 필드가 추가되어야 프론트에서 종료 시간을 표시할 수 있습니다.
            "completed_at": t.completed_at.strftime("%Y-%m-%d %H:%M") if getattr(t, 'completed_at', None) else "-",
            "details": t.details
        }
        for t in tasks
    ]

@router.post("/confirm")
async def confirm_task_action(request: Dict[str, Any], user_id: str = Cookie(None)):
    """
    직원이 로봇으로부터 물품을 수령하거나(RECEIVE), 작업을 취소(CANCEL)하는 요청을 처리합니다.
    """
    task_id = request.get("task_id")
    action_type = request.get("action_type")

    if not task_id:
        raise HTTPException(status_code=400, detail="task_id가 필요합니다.")

    # 1. 해당 작업(Task)이 존재하는지 확인
    task = await container.task_repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="해당 작업을 찾을 수 없습니다.")

    # 2. 사용자 정보 확인
    user = await container.user_repo.get_user_by_username(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    # 3. 권한 확인 (requester_id 필드 사용)
    if task.requester_id != user.user_id:
         raise HTTPException(status_code=403, detail="해당 작업에 대한 권한이 없습니다.")

    # 4. 직접 DB 상태 업데이트 (TaskManager 메서드 부재 해결)
    try:
        new_status = None
        if action_type == 'RECEIVE':
            new_status = "COMPLETED"
        elif action_type == 'CANCEL':
            new_status = "CANCELLED"
        
        if not new_status:
             return {"status": "error", "message": "유효하지 않은 액션 타입입니다."}

        # Repository의 update 메서드를 호출하여 상태 변경
        # mysql_task_repository.py의 update 메서드는 status가 COMPLETED일 때 완료 시간을 자동 기록합니다.
        updated_task = await container.task_repo.update(task_id, {"status": new_status})
        
        if updated_task:
            # (선택 사항) 만약 취소 액션인 경우 로봇에게도 정지 명령을 보내야 한다면 추가
            if action_type == 'CANCEL' and task.assigned_robot_id:
                robot = await container.robot_repo.get_by_id(task.assigned_robot_id)
                if robot:
                    container.fleet_manager.cancel_robot_task(robot.name)

            return {"status": "success", "message": f"작업이 {new_status} 상태로 변경되었습니다."}
        else:
            return {"status": "error", "message": "데이터베이스 업데이트에 실패했습니다."}

    except Exception as e:
        logger.error(f"Confirm Error: {e}")
        return {"status": "error", "message": f"서버 로직 처리 중 오류: {str(e)}"}
    
# ---------------------------------------------------------
# 4. 방문 예약 관리 API (Reservations)
# ---------------------------------------------------------

@router.get("/reservations/pending")
async def get_pending_reservations():
    """승인 대기 목록 조회"""
    visitors = await container.visitor_repository.get_pending_list()
    # 프론트엔드 필터링(내 예약 찾기)을 위해 manager_account(계정명) 추가
    result = []
    for v in visitors:
        v_dict = v.model_dump()
        if v.host_user_id:
            user = await container.user_repo.get_by_id(v.host_user_id)
            if user:
                v_dict['manager_account'] = user.account
        result.append(v_dict)
    return result

@router.get("/reservations/approved")
async def get_approved_reservations():
    """확정된 예약 현황 조회"""
    visitors = await container.visitor_repository.get_approved_list()
    result = []
    for v in visitors:
        v_dict = v.model_dump()
        if v.host_user_id:
            user = await container.user_repo.get_by_id(v.host_user_id)
            if user:
                v_dict['manager_account'] = user.account
        result.append(v_dict)
    return result

@router.get("/reservations/rejected")
async def get_reservations_rejected():
    """반려된 방문 예약 목록 조회"""
    # 1. 리포지토리에서 REJECTED 상태인 데이터를 가져옵니다.
    visitors = await container.visitor_repository.get_rejected_list()
    
    result = []
    for v in visitors:
        v_dict = v.model_dump()
        # 2. 기존 코드와 동일하게 담당자 계정(manager_account) 정보를 매핑합니다.
        if v.host_user_id:
            user = await container.user_repo.get_by_id(v.host_user_id)
            if user:
                v_dict['manager_account'] = user.account
        result.append(v_dict)
        
    return result

@router.get("/locations")
async def get_locations():
    """안내 가능한 목적지 목록 조회"""
    return await container.location_repo.get_all_locations()

@router.post("/reservations/apply")
async def apply_reservation(request: Dict[str, Any]):
    # 1. 담당자 계정으로 User ID 조회 (Visitors 테이블은 host_user_id를 FK로 사용)
    manager_account = request.get('manager_account')
    host_user = await container.user_repo.get_user_by_username(manager_account)
    
    if not host_user:
        raise HTTPException(status_code=400, detail=f"담당자 계정({manager_account})을 찾을 수 없습니다.")

    # 2. 방문객 정보 생성 (Visitors 테이블)
    visitor_data = {
        "host_user_id": host_user.user_id,
        "name": request.get('visitor_name'),
        "phone": request.get('visitor_phone'),
        "purpose": request.get('purpose'),
        "visit_date": request.get('visit_date'),
        "visit_time": request.get('visit_time'),
        "status": "PENDING"
    }
    
    # 목적지 ID가 있다면 추가
    if 'destination_id' in request:
        visitor_data['destination_id'] = request['destination_id']

    res_id = await container.visitor_repository.create(visitor_data)
    
    if not res_id:
        raise HTTPException(status_code=500, detail="예약 생성에 실패했습니다.")

    return {"status": "success", "id": res_id}

# ---------------------------------------------------------
# 5.직원정보
# ---------------------------------------------------------
@router.get("/me")
async def get_my_info(user_id: str = Cookie(None)):
    if not user_id:
        raise HTTPException(status_code=401, detail="인증 정보가 없습니다.")

    user = await container.user_repo.get_user_by_username(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    location_name = "미지정"
    if user.location_id:
        loc = await container.location_repo.get_by_id(user.location_id)
        if loc:
            location_name = loc.name

    # [중요] 이 부분이 안전하게 작성되어야 500 에러가 안 납니다.
    formatted_date = "-"
    if hasattr(user, 'created_at') and user.created_at:
        if isinstance(user.created_at, datetime):
            formatted_date = user.created_at.strftime("%Y-%m-%d")
        else:
            formatted_date = str(user.created_at)[:10]

    return {
        "name": user.name,
        "department": user.department,
        "location": location_name,
        "created_at": formatted_date,
        "role": user.role
    }

# ---------------------------------------------------------
# 6.회의실 예약
# ---------------------------------------------------------

@router.post("/reservations/room")
async def create_room_reservation(request: RoomReservationRequest):
    """회의실 예약 데이터 저장 (중복 체크 포함 통합 버전)"""
    try:
        # 1. 유저 조회
        user = await container.user_repo.get_user_by_username(request.user_id)
        if not user:
            raise HTTPException(status_code=404, detail="사용자 정보를 찾을 수 없습니다.")
        
        user_pk = user.user_id 

        # 2. 중복 예약 확인
        existing_reservations = await container.location_repo.get_room_reservations_by_location(
            request.location_id, str(request.reservation_date)
        )
        
        # 시간 비교를 위한 정규화 (9:00 -> 09:00)
        def normalize_time(t):
            t_str = str(t)
            parts = t_str.split(':')
            return f"{parts[0].zfill(2)}:{parts[1].zfill(2)}"

        req_start = normalize_time(request.start_time)
        req_end = normalize_time(request.end_time)

        for res in existing_reservations:
            if res['status'] == 'CANCLE':
                continue
            
            exist_start = normalize_time(res['start_time'])
            exist_end = normalize_time(res['end_time'])

            # 겹침 조건 확인
            if (req_start < exist_end) and (req_end > exist_start):
                raise HTTPException(
                    status_code=400, 
                    detail=f"이미 해당 시간에 예약이 존재합니다. ({exist_start} ~ {exist_end})"
                )

        # 3. 저장 진행 (Repository 함수 인자 명칭에 맞춤)
        # mysql_location_repository.py의 create_room_reservation 함수 호출
        success = await container.location_repo.create_room_reservation(
            user_pk=user_pk,
            location_id=request.location_id,
            res_date=str(request.reservation_date),
            start_t=request.start_time,
            end_t=request.end_time
        )
        
        if success is not None:
            return {"status": "success", "message": "예약이 완료되었습니다."}
        else:
            raise HTTPException(status_code=500, detail="DB 저장 실패")
            
    except HTTPException as he:
        raise he
    except Exception as e:
        logger.error(f"Error: {e}")
        raise HTTPException(status_code=500, detail=f"서버 오류: {str(e)}")

@router.get("/reservations/room/my")
async def get_my_room_reservations(user_id: str = Cookie(None)):
    if not user_id:
        return []
    try:
        user = await container.user_repo.get_user_by_username(user_id)
        if not user: return []

        reservations = await container.location_repo.get_room_reservations_by_user(user.user_id)
        
        # [중요] 시간/날짜 객체를 문자열로 변환하여 JSON 오류 방지
        for res in reservations:
            if res.get('reservation_date'):
                res['reservation_date'] = str(res['reservation_date'])
            if res.get('start_time'):
                res['start_time'] = str(res['start_time'])
            if res.get('end_time'):
                res['end_time'] = str(res['end_time'])
        
        return reservations
    except Exception as e:
        logger.error(f"Error: {e}")
        return []

@router.patch("/reservations/room/{res_id}/cancel")
async def cancel_room(res_id: int, user_id: str = Cookie(None)):
    user = await container.user_repo.get_user_by_username(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    # 1. DB 업데이트 시도
    result = await container.location_repo.cancel_room_reservation(res_id, user.user_id)
    
    # 2. 판정: result가 0이더라도 에러를 던지지 말고 성공 응답을 보냄
    # 왜냐하면 사용자가 조회를 누르기 전 이미 취소된 상태일 수 있기 때문입니다.
    return {"status": "success", "message": "취소 처리가 완료되었습니다."}