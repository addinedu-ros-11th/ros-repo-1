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
  - `/{robot_ns}/ai_link` (`std_msgs/msg/Bool`)
  - `/{robot_ns}/obstacles` (`std_msgs/msg/String`, optional AI obstacle relay)

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
- Optional obstacle policy in `office_robot_safety`:
  - when `obstacle_enabled=true`, class-based stop/slow thresholds are evaluated from `/{robot_ns}/obstacles`.
  - current runtime behavior: `STOP` threshold triggers safety lock; `SLOW` threshold is observability/log state only.
  - final lock is `command_lock OR obstacle_lock` to keep STOP/PAUSE semantics deterministic.
- Nav2 startup recovery in `office_robot_executor`:
  - executor validates localization readiness (`amcl_pose` freshness/covariance, optional `map->odom` TF) before Nav2 goal send.
  - executor also gates by Nav2 lifecycle state (`planner/controller/bt/behavior` must be `active`).
  - when blocked, `/{robot_ns}/event` emits `LOCALIZATION_NOT_READY` with machine-readable reason.
  - if not ready, recovery cycle can run:
    - call `/{robot_ns}/reinitialize_global_localization` (service name configurable)
    - rotate in place (`cmd_vel`) for active scan
    - request Nav2 lifecycle manager `STARTUP/RESUME` if lifecycle nodes are inactive
    - retry with bounded attempts (`nav2_retry_*`, `localization_recovery_*`, `amcl_*` params).
- Obstacle and other-robot avoidance is handled by Nav2 costmap/controller policy.
- AI dependency split:
  - AI-independent actions can still execute while AI is down.
  - For AI-dependent actions, upper layer should check `/{robot_ns}/ai_link` or
    `ai_link_alive` in `/{robot_ns}/status` before issuing commands.

## Runtime Launch
```bash
cd /home/changpc/ros-repo-1/robot/jazzy_ws
source install/setup.bash
ros2 launch office_robot_bringup bringup.launch.py \
  robot_ns:=robot01 robot_id:=1 enable_rosbridge:=true \
  use_nav2:=true nav2_action_name:=/robot01/navigate_to_pose
```

## Nav2 Params File Fix Checklist
```bash
# 1) audit current runtime (before)
/home/pinky/ros-repo-1/robot/scripts/nav2_runtime_audit.sh

# 2) install systemd override template (root)
sudo /home/pinky/ros-repo-1/robot/scripts/install_pinky_navigation_override.sh

# 3) ensure runtime env has map + params path
grep -E '^(MAP_PATH|NAV2_PARAMS_FILE)=' /etc/robot_runtime.env

# 4) restart navigation service and re-check
sudo systemctl restart pinky-navigation.service
/home/pinky/ros-repo-1/robot/scripts/nav2_runtime_audit.sh
```

- Required in `/etc/robot_runtime.env`:
  - `MAP_PATH=/home/pinky/.../*.yaml`
  - `NAV2_PARAMS_FILE=/home/pinky/pinky_pro/install/pinky_navigation/share/pinky_navigation/params/nav2_params.yaml`
- Do not run `pinky_navigation/launch/navigation_launch.xml` standalone for production bringup.
  It can fall back to `nav2_bringup` default params if `params_file` is not explicitly chained.

## Camera + UDP Autostart (systemd)
```bash
# template deployment (on robot)
sudo cp /home/pinky/ros-repo-1/robot/systemd/robot-camera.service /etc/systemd/system/
sudo cp /home/pinky/ros-repo-1/robot/systemd/robot-udp-bridge.service /etc/systemd/system/
sudo cp /home/pinky/ros-repo-1/robot/systemd/robot_runtime.env.example /etc/robot_runtime.env
sudo systemctl daemon-reload
sudo systemctl enable --now robot-udp-bridge.service
# topic source mode only:
# sudo systemctl enable --now robot-camera.service
```

```bash
# runtime checks
systemctl --no-pager --full status robot-camera.service
systemctl --no-pager --full status robot-udp-bridge.service
grep -E '^CAMERA_SOURCE=' /etc/robot_runtime.env
# topic mode only:
# ros2 topic info /camera/image_raw -v
# ros2 topic hz /camera/image_raw
journalctl -u robot-udp-bridge.service -n 50 --no-pager
```

## Verification Checklist
```bash
ros2 topic info /robot01/commands -v
ros2 topic info /robot01/status -v
ros2 topic info /robot01/event -v
ros2 topic echo /robot01/ai_link --once
ss -lntp | grep 9090
```

## RViz Debug (On-PC)
```bash
# PC must be on same network / ROS_DOMAIN_ID as robot
cd /home/changpc/ros-repo-1
ROBOT_NS=robot01 ROS_DOMAIN_ID=88 ./robot/scripts/run_rviz_nav_debug.sh

# or direct launch
ros2 launch office_robot_bringup nav_debug_rviz.launch.py robot_ns:=robot01
```

## Compatibility Notes
- Keep topic/port contract stable; downstream services depend on it.
- Any schema/key changes must be documented in `HANDOFF.md` and communicated before rollout.
- For Nav2 startup, always pin `params_file` in service/launch chain.
