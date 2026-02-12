from enum import Enum

class RobotEvent(str, Enum):
    """
    로봇으로부터 수신되는 세부 작업 완료 및 상태 변경 이벤트 정의.
    상위 common 폴더에 위치하여 시스템 전반에서 공통 규약으로 사용됩니다.
    """
    # 공통 및 기본 이동 관련
    ARRIVED_AT_DESTINATION = "ARRIVED_AT_DESTINATION"
    ARRIVED_AT_BASE = "ARRIVED_AT_BASE"
    
    # 간식 배달 시나리오 (Snack Delivery)
    ARRIVED_AT_PANTRY_ENTRANCE = "ARRIVED_AT_PANTRY_ENTRANCE"
    ARRIVED_AT_SNACK_POINT = "ARRIVED_AT_SNACK_POINT"
    
    # 가이드 시나리오 (Guide)
    # (기본적으로 ARRIVED_AT_DESTINATION, ARRIVED_AT_BASE 활용)

    # 물품 배달 시나리오 (Item Delivery)
    ARRIVED_AT_SENDER = "ARRIVED_AT_SENDER"
    LOADING_COMPLETE = "LOADING_COMPLETE"
    ARRIVED_AT_RECEIVER = "ARRIVED_AT_RECEIVER"
    DELIVERY_CONFIRMED = "DELIVERY_CONFIRMED"
