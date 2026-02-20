# Robot Handoff (ROS 2 Jazzy)

## Definition of Done (DoD)
- `robot-han/jazzy_ws` exists and `colcon build` succeeds.
- This workspace is intended for ROS 2 Jazzy only.
- `office_robot_bringup` launch brings up the executor (and optional rosbridge).
- Namespaced topics exist per robot:
  - `/<robot_ns>/commands` (std_msgs/String)
  - `/<robot_ns>/status` (std_msgs/String)
  - `/<robot_ns>/event` (std_msgs/String)

## Namespace / Multi-Robot
- Use `robot_ns` launch argument (default `robot_a`).
- Example namespaces: `/robot_a`, `/robot_b`.
- All executor topics live under the namespace via `PushRosNamespace`.

## Topic Contracts (v0, mock)
- `commands` (String JSON):
  - Example: `{"robot_name":"robot","type":"ACTION_SEQUENCE","payload":[{"action":"GOTO","params":{"x":1.2,"y":-0.4},"on_success":"ARRIVED_AT_DESTINATION"}]}`
- `status` (String JSON):
  - Example: `{"robot_id":1,"robot_name":"robot","status":"MOVING","location":[1.2,-0.4],"battery":100.0,"event":"ARRIVED_AT_DESTINATION"}`
- `event` (String JSON):
  - Example: `{"robot_id":1,"robot_name":"robot","event":"ARRIVED_AT_DESTINATION","task_id":1}`

## How to Run
```bash
cd robot-han/jazzy_ws
colcon build
source install/setup.bash
ros2 launch office_robot_bringup bringup.launch.py robot_ns:=robot_a
```

## Quick Test (publish a task)
```bash
ros2 topic pub /robot/commands std_msgs/String "{data: '{\"robot_name\":\"robot\",\"type\":\"ACTION_SEQUENCE\",\"payload\":[{\"action\":\"GOTO\",\"params\":{\"x\":2.0,\"y\":3.0},\"on_success\":\"ARRIVED_AT_DESTINATION\"}] }'}"
```
