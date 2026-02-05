from typing import List, Dict, Any, Optional
from fastapi import APIRouter, Depends, Body, HTTPException

from main_server.domains.tasks.task import Task, TaskType
from main_server.core_layer.task_management.task_manager import TaskManager
from main_server.container import container

# --- 요청 본문 모델 ---
from pydantic import BaseModel

class CreateTaskRequest(BaseModel):
    task_type: TaskType
    requester_id: int
    details: Dict[str, Any] = {}

# --- FastAPI 라우터 생성 ---
router = APIRouter(
    prefix="/api/v1/employee",
    tags=["Employee"],
)

@router.post("/tasks", response_model=Task, status_code=201)
async def create_new_task(
    task_request: CreateTaskRequest = Body(...),
    task_manager: TaskManager = Depends(lambda: container.task_manager)
):
    """
    새로운 작업을 요청합니다. (간식, 배달 등) (SR-009)
    TaskManager를 호출하여 작업을 생성하고 로봇 할당을 시도합니다.
    """
    if not task_manager:
        raise HTTPException(status_code=503, detail="Task service is not available.")
    
    try:
        created_task = await task_manager.create_new_task(
            task_type=task_request.task_type,
            requester_id=task_request.requester_id,
            details=task_request.details
        )
        return created_task
    except Exception as e:
        # 실제 운영 환경에서는 에러 로깅이 필요합니다.
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred: {e}")

@router.get("/tasks/{task_id}", response_model=Optional[Task])
async def get_task_status(
    task_id: int,
    task_manager: TaskManager = Depends(lambda: container.task_manager)
):
    """
    특정 작업의 현재 상태를 조회합니다.
    """
    if not task_manager:
        raise HTTPException(status_code=503, detail="Task service is not available.")
    
    task = await task_manager.get_task_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Task with ID {task_id} not found.")
    return task