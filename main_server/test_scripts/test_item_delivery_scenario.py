import requests
import json
import time

BASE_URL = "http://localhost:8000"

def log(msg):
    print(f"[TEST] {msg}")

def test_item_delivery():
    # 0. Load Dummy Map (ALL WHITE)
    log("Loading dummy map...")
    resp = requests.post(f"{BASE_URL}/api/test/load_map", params={"map_path": "test_map.yaml"})
    if resp.status_code != 200:
        log(f"Failed to load map: {resp.text}")
        return
    log(f"Map loaded: {resp.json()}")

    # 1. Update Locations (Set to safe center points)
    log("Updating locations to safe coordinates...")
    # Alice's office (office_1) at (2.0, 2.0)
    requests.put(f"{BASE_URL}/api/test/locations/office_1", params={"x": 2.0, "y": 2.0})
    # Bob's office (office_2) at (8.0, 8.0)
    requests.put(f"{BASE_URL}/api/test/locations/office_2", params={"x": 8.0, "y": 8.0})

    # 2. 테스트용 사용자 생성
    log("Creating test users Alice and Bob...")
    # Alice (Requester) - Matching test_snack_scenario account
    requests.post(f"{BASE_URL}/api/test/users", json={
        "account": "test_user_alice", "password": "password123", "name": "Alice", "role": "STAFF", "location_id": 1
    })
    # Bob (Receiver) - name Bob for AI matching
    requests.post(f"{BASE_URL}/api/test/users", json={
        "account": "test_user_bob", "password": "password123", "name": "Bob", "role": "STAFF", "location_id": 2
    })

    # 3. 테스트용 로봇 생성
    log("Creating/Updating test robot...")
    requests.post(f"{BASE_URL}/api/test/robots", json={
        "name": "item_bot", "initial_x": 5.0, "initial_y": 5.0
    })

    # 4. 로그인하여 쿠키 획득 (Alice로 로그인)
    log("Logging in as 'test_user_alice'...")
    login_payload = {"username": "test_user_alice", "password": "password123"}
    session = requests.Session()
    resp = session.post(f"{BASE_URL}/api/v1/login", data=login_payload)
    
    if resp.status_code != 200:
        log(f"Login failed: {resp.text}")
        return
    log(f"Login successful. Cookies: {session.cookies.get_dict()}")

    # 5. 물품 배달 주문 (Bob에게 전달 요청)
    log("Sending item delivery command: 'Bob에게 이거 좀 전해줘'...")
    command_payload = {
        "message": "Bob에게 이거 좀 전해줘", 
        "user_id": "test_user_alice" 
    }
    
    resp = session.post(f"{BASE_URL}/api/v1/employee/command", json=command_payload)
    
    if resp.status_code != 200:
        log(f"Command failed: {resp.text}")
        return

    result = resp.json()
    log(f"Command result: {result['status']} - {result['message']}")
    task_id = result.get("task_id")
    
    if not task_id:
        log("ERROR: No task ID created. Check server logs for path planning or AI parsing issues.")
        return

    log(f"SUCCESS: Task created with ID: {task_id}")

    # 6. 로봇 이벤트 시뮬레이션
    # ITEM_DELIVERY 흐름: SENDER 도착 -> 로딩 확인 -> RECEIVER 도착 -> 수령 확인
    
    # 6-1. 발송인(Alice)에게 도착
    log("Simulating: Robot arrived at Sender (Alice)...")
    time.sleep(1) 
    resp = session.post(f"{BASE_URL}/api/test/tasks/{task_id}/events", json={"task_id": task_id, "event_type": "ARRIVED_AT_SENDER"})
    if resp.status_code != 200:
        log(f"Failed to trigger ARRIVED_AT_SENDER: {resp.text}")
        return

    # 6-2. 물품 적재 확인 (Alice가 확인 버튼 클릭)
    log("Simulating: Sender confirms loading...")
    time.sleep(1)
    confirm_loading_payload = {
        "task_id": task_id,
        "action_type": "CONFIRM_LOADING"
    }
    resp = session.post(f"{BASE_URL}/api/v1/employee/confirm", json=confirm_loading_payload)
    if resp.status_code != 200:
        log(f"Failed to confirm loading: {resp.text}")
        return

    # 6-3. 수신인(Bob)에게 도착
    log("Simulating: Robot arrived at Receiver (Bob)...")
    time.sleep(1)
    resp = session.post(f"{BASE_URL}/api/test/tasks/{task_id}/events", json={"task_id": task_id, "event_type": "ARRIVED_AT_RECEIVER"})
    if resp.status_code != 200:
        log(f"Failed to trigger ARRIVED_AT_RECEIVER: {resp.text}")
        return

    # 6-4. 물품 수령 확인 (Bob이 확인 버튼 클릭)
    log("Final Step: Confirming receipt at Receiver...")
    time.sleep(1)
    confirm_receipt_payload = {
        "task_id": task_id,
        "action_type": "CONFIRM_ITEM_RECEIPT"
    }
    resp = session.post(f"{BASE_URL}/api/v1/employee/confirm", json=confirm_receipt_payload)
    
    if resp.status_code == 200:
        log(f"Receipt confirmed! Item delivery test finished successfully.")
    else:
        log(f"Failed to confirm receipt: {resp.text}")

if __name__ == "__main__":
    try:
        test_item_delivery()
    except requests.exceptions.ConnectionError:
        log("Connection error. Ensure the main server is running on localhost:8000")
