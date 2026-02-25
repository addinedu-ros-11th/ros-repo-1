# Robot Integration Guide (Current Runtime Contract)

This guide documents the integration contract for the robot runtime in
`robot/jazzy_ws`.

## Roles
- `main_server`: decides high-level behavior and sends robot commands.
- `ai_server`: runs vision/AI inference and returns results to upper layer.
- `robot` runtime: executes movement/safety actions from commands.

## Network / IPC Contract
- rosbridge: `9090/tcp`
- UDP camera stream: `54321/udp` (Robot -> AI vision receiver)
- ROS topics:
  - `/{robot_ns}/commands`
  - `/{robot_ns}/status`
  - `/{robot_ns}/event`

## Command Model
- Message type: `std_msgs/msg/String` with JSON payload.
- Core commands:
  - `ACTION_SEQUENCE` (`GOTO`, `LEAD_GUEST`, `DISPLAY_TEXT`, `PAUSE`, `RESUME`)
  - top-level `STOP`, `PAUSE`, `RESUME`, `CANCEL`

## Safety Model
- Person detection decision is made by upper layer (`main_server` / AI pipeline).
- Robot-side execution:
  - `STOP`/`PAUSE`: safety lock on + active goal cancel + zero velocity hold
  - `RESUME`: safety lock off + next action sequence allowed
- Obstacle and other-robot avoidance is handled by Nav2 costmap/controller policy.

## Runtime Launch
```bash
cd /home/changpc/ros-repo-1/robot/jazzy_ws
source install/setup.bash
ros2 launch office_robot_bringup bringup.launch.py \
  robot_ns:=robot_1 robot_id:=1 enable_rosbridge:=true \
  use_nav2:=true nav2_action_name:=/robot_1/navigate_to_pose
```

## Verification Checklist
```bash
ros2 topic info /robot_1/commands -v
ros2 topic info /robot_1/status -v
ros2 topic info /robot_1/event -v
ss -lntp | grep 9090
```

## Compatibility Notes
- Keep topic/port contract stable; downstream services depend on it.
- Any schema/key changes must be documented in `HANDOFF.md` and communicated before rollout.
