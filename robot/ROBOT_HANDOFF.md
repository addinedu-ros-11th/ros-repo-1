# Robot Handoff (ROS 2 Jazzy)

## Definition of Done (DoD)
- `robot-han/jazzy_ws` exists and `colcon build` succeeds.
- This workspace is intended for ROS 2 Jazzy only.
- `office_robot_bringup` launch brings up the executor (and optional rosbridge).
- Namespaced topics exist per robot:
  - `/<robot_ns>/task` (std_msgs/String)
  - `/<robot_ns>/status` (std_msgs/String)
  - `/<robot_ns>/event` (std_msgs/String)

## Namespace / Multi-Robot
- Use `robot_ns` launch argument (default `robot_a`).
- Example namespaces: `/robot_a`, `/robot_b`.
- All executor topics live under the namespace via `PushRosNamespace`.

## Topic Contracts (v0, mock)
- `task` (String JSON):
  - Example: `{"task_id": 1, "task_type": "item_delivery", "destination": {"x": 1.2, "y": -0.4}}`
- `status` (String JSON):
  - Example: `{"robot": "robot_a", "state": "in_progress", "task_id": 1}`
- `event` (String JSON):
  - Example: `{"robot": "robot_a", "event": "task_completed", "task_id": 1}`

## How to Run
```bash
cd robot-han/jazzy_ws
colcon build
source install/setup.bash
ros2 launch office_robot_bringup bringup.launch.py robot_ns:=robot_a
```

## Quick Test (publish a task)
```bash
ros2 topic pub /robot_a/task std_msgs/String "{data: '{\"task_id\": 1, \"task_type\": \"snack_delivery\", \"destination\": {\"x\": 2.0, \"y\": 3.0}}'}"
```
