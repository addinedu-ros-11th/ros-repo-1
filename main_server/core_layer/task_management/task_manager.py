from typing import Dict, Any, Optional

from main_server.domains.tasks.task import Task, TaskType, TaskStatus
from main_server.domains.tasks.task_repository import ITaskRepository
from main_server.core_layer.fleet_management.fleet_manager import FleetManager
from main_server.common.exceptions import TaskAssignmentException

class TaskManager:
    """
    사용자로부터의 작업 요청을 생성, 처리하고 FMS에 할당을 요청하는 비동기 서비스.
    이것이 아키텍처 문서의 'Task Management Service'입니다.
    """
    def __init__(self, task_repo: ITaskRepository, fleet_manager: FleetManager):
        self.task_repo = task_repo
        self.fleet_manager = fleet_manager
        print("TaskManager initialized.")

    async def create_new_task(self, task_type: TaskType, requester_id: int, details: Dict[str, Any]) -> Task:
        """
        새로운 작업을 생성하고, 즉시 할당을 시도합니다. (SR-009)
        """
        task_data = {
            "task_type": task_type,
            "requester_id": requester_id,
            "details": details,
            "status": TaskStatus.PENDING
        }
        new_task = await self.task_repo.create(task_data)
        print(f"New task created with ID: {new_task.id}")
        
        # 즉시 할당 시도
        assigned_task = await self.try_to_assign_task(new_task)
        return assigned_task or new_task

    async def try_to_assign_task(self, task: Task) -> Optional[Task]:
        """
        주어진 작업을 최적의 로봇에 할당 시도합니다.
        성공하면 업데이트된 Task 객체를, 실패하면 None을 반환합니다.
        """
        try:
            # 1. 최적 로봇 탐색 (목적지 정보가 필요하다면 details에서 추출)
            # 예시: 'destination' 키가 있으면 그걸 쓰고, 없으면 기본값 (0,0) 사용
            destination = task.details.get("destination", {"x": 0, "y": 0})
            target_pose = (destination.get("x", 0), destination.get("y", 0))
            
            optimal_robot = await self.fleet_manager.find_optimal_robot(target_pose)
            
            if optimal_robot:
                # 2. 로봇에게 작업 할당 (FMS) - 수정된 시그니처에 맞게 호출
                await self.fleet_manager.assign_task_to_robot(optimal_robot, task)

                # 3. 작업 상태를 'ASSIGNED'로 업데이트 (Task Repo)
                update_data = {"status": TaskStatus.ASSIGNED, "robot_id": optimal_robot.id}
                updated_task = await self.task_repo.update(task.id, update_data)

                # 4. 사용자에게 알림 (Notification Service 호출 - 미구현)
                print(f"Notification: Task {task.id} assigned to robot {optimal_robot.name}.")
                return updated_task
            else:
                print(f"No available robot for task {task.id}. It remains in PENDING state.")
                return None

        except Exception as e:
            print(f"Failed to assign task {task.id}: {e}")
            # 할당 실패 시 작업 상태를 'FAILED'로 업데이트
            await self.task_repo.update(task.id, {"status": TaskStatus.FAILED})
            raise TaskAssignmentException(f"Failed to assign task {task.id}: {e}") from e

    async def process_pending_tasks(self):
        """
        주기적으로 호출되어, 처리되지 않은 'PENDING' 상태의 작업들을 할당 시도합니다.
        """
        pending_tasks = await self.task_repo.get_all_by_status(TaskStatus.PENDING)
        if not pending_tasks:
            return

        print(f"Found {len(pending_tasks)} pending tasks. Trying to assign...")
        for task in pending_tasks:
            await self.try_to_assign_task(task)

    async def get_task_by_id(self, task_id: int) -> Optional[Task]:
        """ID로 작업을 조회합니다."""
        return await self.task_repo.find_by_id(task_id)

