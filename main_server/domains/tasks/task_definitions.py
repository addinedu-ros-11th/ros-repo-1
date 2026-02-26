from pydantic import BaseModel, Field
from typing import List, Dict, Any

class TaskScenario(BaseModel):
    """Defines the fields expected from the AI for each task type"""
    pass

class SnackDeliveryScenario(TaskScenario):
    requester_name: str
    items: List[Dict[str, Any]] = Field(..., description="List of items, e.g., [{'item_name': 'coffee', 'quantity': 2}]")
    
class ItemDeliveryScenario(TaskScenario):
    requester_name: str
    receiver_name: str
    items: List[Dict[str, Any]]

class GuideGuestScenario(TaskScenario):
    visitor_name: str
    destination: str # e.g., 'small_meeting_room'

# Mapping from TaskType enum to the scenario model and metadata
TASK_SCENARIO_MAP = {
    "SNACK_DELIVERY": {
        "model": SnackDeliveryScenario,
        "initial_destination": "snack_entrance", # 간식 창고 입구
        "priority": 3,
    },
    "ITEM_DELIVERY": {
        "model": ItemDeliveryScenario,
        # 초기 목적지가 요청자 위치로 동적이므로 هنا서는 제거
        "priority": 2,
    },
    "GUIDE_GUEST": {
        "model": GuideGuestScenario,
        "initial_destination": "waiting_area", # 방문객 대기 장소 (가정)
        "priority": 1,
    },
    "MANUAL_MOVE": {
        "model": TaskScenario, # 별도 필드 검증 없이 통과
        "initial_destination": None, # 좌표가 직접 주어짐
        "priority": 1,
    }
}
