import requests
import json
import time

BASE_URL = "http://localhost:8000"

def log(msg):
    print(f"[TEST] {msg}")

def test_scenario():
    # 0. Load Dummy Map (ALL WHITE)
    log("Loading dummy map...")
    resp = requests.post(f"{BASE_URL}/api/test/load_map", params={"map_path": "test_map.yaml"})
    if resp.status_code != 200:
        log(f"Failed to load map: {resp.text}")
        return
    log(f"Map loaded: {resp.json()}")

    # 1. Update Locations (Set to safe center points)
    log("Updating locations to safe coordinates...")
    # snack_entrance at (5.0, 5.0)
    requests.put(f"{BASE_URL}/api/test/locations/snack_entrance", params={"x": 5.0, "y": 5.0})
    # office_1 (Alice's home) at (8.0, 8.0)
    requests.put(f"{BASE_URL}/api/test/locations/office_1", params={"x": 8.0, "y": 8.0})

    # 2. 테스트용 사용자 생성 (Alice: office_1)
    log("Creating test user 'test_user_alice'...")
    user_data = {
        "account": "test_user_alice",
        "password": "password123",
        "name": "Alice",
        "role": "STAFF",
        "location_id": 1 # Assume office_1 is ID 1
    }
    resp = requests.post(f"{BASE_URL}/api/test/users", json=user_data)
    log(f"User creation/check: {resp.json()}")

    # 3. 테스트용 로봇 2대 생성 (Center positions)
    log("Creating/Updating test robots...")
    robots = [
        {"name": "robot_test_1", "initial_x": 1.0, "initial_y": 1.0},
        {"name": "robot_test_2", "initial_x": 9.0, "initial_y": 9.0}
    ]
    
    for r in robots:
        resp = requests.post(f"{BASE_URL}/api/test/robots", json=r)
        if resp.status_code == 200:
            log(f"Robot {r['name']} status: {resp.json()}")
        else:
            log(f"Failed to create robot {r['name']}: {resp.text}")

    # 4. 로그인하여 쿠키 획득
    log("Logging in as 'test_user_alice'...")
    login_payload = {"username": "test_user_alice", "password": "password123"}
    session = requests.Session()
    resp = session.post(f"{BASE_URL}/api/v1/login", data=login_payload)
    
    if resp.status_code != 200:
        log(f"Login failed: {resp.text}")
        return

    log(f"Login successful. Cookies: {session.cookies.get_dict()}")

    # 5. 간식 배달 주문
    log("Sending snack delivery command (Requesting coffee)...")
    command_payload = {
        "message": "커피 배달해줘", 
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
        log("ERROR: No task ID. Check if robots were found as IDLE and had valid paths.")
        # Check current robot status for debugging
        # We can add a debug endpoint for fleet status if needed
        return

    log(f"SUCCESS: Task created with ID: {task_id}")

    # 6. 로봇 이벤트 시뮬레이션
    events = [
        "ARRIVED_AT_PANTRY_ENTRANCE",
        "ARRIVED_AT_SNACK_POINT",
        "ARRIVED_AT_DESTINATION"
    ]

    for event in events:
        log(f"Simulating robot event: {event}...")
        time.sleep(1) 
        resp = session.post(f"{BASE_URL}/api/test/tasks/{task_id}/events", json={"task_id": task_id, "event_type": event})
        if resp.status_code == 200:
            log(f"Event {event} processed.")
        else:
            log(f"Failed to trigger event {event}: {resp.text}")
            break

    # 7. 최종 수령 확인
    log("Final Step: Confirming receipt...")
    time.sleep(1)
    confirm_payload = {
        "task_id": task_id,
        "action_type": "CONFIRM_SNACK_RECEIPT"
    }
    resp = session.post(f"{BASE_URL}/api/v1/employee/confirm", json=confirm_payload)
    
    if resp.status_code == 200:
        log(f"Receipt confirmed! Test finished successfully.")
    else:
        log(f"Failed to confirm receipt: {resp.text}")

if __name__ == "__main__":
    try:
        test_scenario()
    except requests.exceptions.ConnectionError:
        log("Connection error. Ensure the main server is running on localhost:8000")
