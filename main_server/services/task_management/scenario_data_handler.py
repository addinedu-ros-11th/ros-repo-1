import logging
from typing import Any, Dict, Optional

from main_server.domains.tasks.task_definitions import TASK_SCENARIO_MAP
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository

logger = logging.getLogger(__name__)

class ScenarioDataHandler:
    """AI 결과를 해석하여 DB에 저장할 Task 데이터와 로봇 배차에 필요한 정보를 생성합니다."""
    
    def __init__(self, 
                 location_repo: MySQLLocationRepository, 
                 user_repo: MySQLUserRepository, 
                 product_repo: MySQLProductRepository):
        self.location_repo = location_repo
        self.user_repo = user_repo
        self.product_repo = product_repo

    async def prepare_task_data(self, ai_result: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """AI 결과로부터 태스크 생성에 필요한 데이터를 준비합니다."""
        task_type = ai_result.get("task_type")
        if not task_type or task_type not in TASK_SCENARIO_MAP:
            logger.error(f"Unsupported task type: {task_type}")
            return None

        scenario_info = TASK_SCENARIO_MAP[task_type]
        fields = ai_result.get("fields", {})

        try:
            if task_type == "SNACK_DELIVERY":
                return await self._prepare_snack_delivery(fields, scenario_info)
            elif task_type == "ITEM_DELIVERY":
                return await self._prepare_item_delivery(fields, scenario_info)
            else:
                logger.warning(f"Task type '{task_type}' handler not fully implemented yet.")
                return None
        except Exception as e:
            logger.error(f"Failed to process AI result for {task_type}: {e}", exc_info=True)
            return None

    async def _prepare_snack_delivery(self, fields: Dict[str, Any], scenario_info: Dict) -> Optional[Dict]:
        """간식 배달 시나리오 데이터를 준비합니다."""
        requester_name = fields.get("requester_name")
        if not requester_name:
            logger.error("Requester name not found in AI result for SNACK_DELIVERY")
            return None

        requester = await self.user_repo.find_by_name(requester_name)
        if not requester:
            logger.error(f"Requester '{requester_name}' not found in DB.")
            return None

        # Use location_id from user record to find destination
        if not requester.get('location_id'):
            logger.error(f"User '{requester_name}' has no assigned location_id.")
            return None

        final_destination = await self.location_repo.find_by_id(requester['location_id'])
        if not final_destination:
            logger.error(f"Location ID {requester['location_id']} for user '{requester_name}' not found.")
            return None

        task_items = []
        ai_items = fields.get("items", [])
        for item in ai_items:
            product = await self.product_repo.find_by_name(item.get("item_name"))
            if product:
                task_items.append({"product_id": product['product_id'], "quantity": item.get("quantity", 1)})
            else:
                logger.warning(f"Product '{item.get('item_name')}' not found in DB.")

        task_data = {
            "requester_id": requester['user_id'],
            "task_type": "SNACK_DELIVERY",
            "priority": scenario_info["priority"],
            "status": "PENDING",
            "destination_id": final_destination['location_id'],
            "target_location_name": final_destination['name'],
            "details": fields
        }

        return {
            "task_data": task_data,
            "task_items": task_items,
            "initial_destination_name": scenario_info["initial_destination"]
        }

    async def _prepare_item_delivery(self, fields: Dict[str, Any], scenario_info: Dict) -> Optional[Dict]:
        """물품 배달 시나리오 데이터를 준비합니다."""
        requester_name = fields.get("requester_name")
        receiver_name = fields.get("receiver_name")

        if not requester_name or not receiver_name:
            logger.error("Requester and Receiver names are required for ITEM_DELIVERY")
            return None

        requester = await self.user_repo.find_by_name(requester_name)
        if not requester:
            logger.error(f"Requester '{requester_name}' not found.")
            return None
        
        # Find source location (requester's location)
        if not requester.get('location_id'):
            logger.error(f"Requester '{requester_name}' has no assigned location_id.")
            return None

        initial_destination = await self.location_repo.find_by_id(requester['location_id'])
        if not initial_destination:
            logger.error(f"Location ID {requester['location_id']} for requester '{requester_name}' not found.")
            return None

        receiver = await self.user_repo.find_by_name(receiver_name)
        if not receiver:
            logger.error(f"Receiver '{receiver_name}' not found.")
            return None
            
        # Find destination location (receiver's location)
        if not receiver.get('location_id'):
            logger.error(f"Receiver '{receiver_name}' has no assigned location_id.")
            return None

        final_destination = await self.location_repo.find_by_id(receiver['location_id'])
        if not final_destination:
            logger.error(f"Location ID {receiver['location_id']} for receiver '{receiver_name}' not found.")
            return None

        # TaskProcessor가 출발지를 알 수 있도록 details에 명시
        fields["source_location"] = initial_destination['name']

        task_data = {
            "requester_id": requester['user_id'],
            "receiver_id": receiver['user_id'],
            "task_type": "ITEM_DELIVERY",
            "priority": scenario_info["priority"],
            "status": "PENDING",
            "destination_id": final_destination['location_id'],
            "target_location_name": final_destination['name'],
            "details": fields
        }
        
        return {
            "task_data": task_data,
            "task_items": [],  # User-to-user delivery doesn't use Task_Items
            "initial_destination_name": initial_destination['name']
        }
