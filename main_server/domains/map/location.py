from enum import Enum
from typing import Dict, NamedTuple

# 1. 위치 이름을 Enum으로 정의 (오타 방지 및 코드 가독성)
class LocationName(Enum):
    OFFICE_1 = "office_1"
    OFFICE_2 = "office_2"
    OFFICE_3 = "office_3"
    SNACK_BAR = "snack_bar"
    SMALL_MEETING_ROOM = "small_meeting_room"
    LARGE_MEETING_ROOM = "large_meeting_room"
    CHARGER_1 = "charger_1"
    CHARGER_2 = "charger_2"

# 2. 좌표를 저장할 구조 정의 (x, y 미터 단위)
class Pose(NamedTuple):
    x: float
    y: float
    theta: float

# 3. 실제 좌표 데이터 정의 (사무실 환경에 맞춰 미터 단위로 설정)
# Tip: SLAM으로 맵을 그린 후 Rviz 등에서 확인한 좌표값을 입력하세요.
WAYPOINTS: Dict[LocationName, Pose] = {
    LocationName.OFFICE_1: Pose(x=0.5, y=1.2, theta=0.0),
    LocationName.OFFICE_2: Pose(x=1.5, y=1.2, theta=0.0),
    LocationName.OFFICE_3: Pose(x=2.5, y=1.2, theta=0.0),
    LocationName.SNACK_BAR: Pose(x=4.0, y=0.5, theta=0.0),
    LocationName.SMALL_MEETING_ROOM: Pose(x=3.5, y=3.0, theta=0.0),
    LocationName.LARGE_MEETING_ROOM: Pose(x=1.0, y=3.0, theta=0.0),
    LocationName.CHARGER_1: Pose(x=0.1, y=0.1, theta=0.0),
    LocationName.CHARGER_2: Pose(x=4.8, y=0.1, theta=0.0),
}