# Robot Handoff (ROS 2 Jazzy)

## Scope
- This document covers only `robot/` runtime behavior and ops commands.
- `main_server/` and `ai_server/` are out of scope.

## Definition of Done (Current)
- `robot/jazzy_ws` builds successfully (`colcon build --symlink-install`).
- `office_robot_bringup` launches executor + safety (+ optional rosbridge).
- Namespaced topics exist:
  - `/{robot_ns}/commands` (`std_msgs/msg/String`)
  - `/{robot_ns}/status` (`std_msgs/msg/String`)
  - `/{robot_ns}/event` (`std_msgs/msg/String`)
  - `/{robot_ns}/ai_link` (`std_msgs/msg/Bool`, AI link health)
  - `/{robot_ns}/obstacles` (`std_msgs/msg/String`, obstacle relay input)
  - `/{robot_ns}/safety_state` (`std_msgs/msg/String`, latched safety snapshot)

## Namespace / Multi-Robot
- Default namespace: `robot01`
- Launch arg: `robot_ns` (example: `robot01`, `robot02`)
- All robot runtime nodes are launched under `PushRosNamespace(robot_ns)`.

## Command / Status Contract
- Command envelope: JSON string on `/{robot_ns}/commands`
- Supported top-level command `type`:
  - `ACTION_SEQUENCE`
  - `STOP`
  - `PAUSE`
  - `RESUME`
  - `CANCEL`
- `task_id`/`sequence_id` are both carried by executor for compatibility.
  - Safety lock behavior:
    - `STOP`/`PAUSE` => lock enabled, `cmd_vel` zero hold, running goal canceled.
    - `RESUME` => lock released, new action sequence can run.
  - Optional obstacle policy (`office_robot_safety`):
    - subscribes `/{robot_ns}/obstacles` and evaluates dynamic obstacle policy.
    - `person` / `robot`: presence-only `STOP` by default even when upstream distance is missing.
    - `chair` / `plant` / `bag`: distance-based threshold only; without distance they fall back to Nav2 static avoidance.
    - runtime today: `STOP` threshold enforces lock/zero-velocity, `SLOW` threshold is state/log only.
    - lock source is merged (`command_lock OR obstacle_lock`) to avoid accidental unlock.
  - `/{robot_ns}/status` may carry latest safety metadata:
    - `event`
    - `safety_source`
    - `obstacle_state`
    - `obstacle_class`
    - `obstacle_confidence`
    - `obstacle_distance`
    - `obstacle_box`
    - `obstacle_reason`
  - `SAFETY_STOPPED` / `SAFETY_RESUMED` are transition events only.
    A latched `/{robot_ns}/safety_state` snapshot may refresh status metadata, but it must not be
    treated as a new resume/stop transition unless the safety lock actually changed.
  - Battery fields:
    - `battery`
    - `battery_valid`
    - `battery_source_topic`
    - `battery_error` (`battery_topic_unavailable` if `/battery/present` has no publisher/data)
  - Nav2 recovery behavior (`office_robot_executor`):
    - validates localization readiness (`amcl_pose`, covariance, optional `map->odom` TF) before goal send.
    - validates required Nav2 lifecycle nodes are `active` before goal send
      (`planner_server`, `controller_server`, `bt_navigator`, `behavior_server` by default).
    - startup bootstrap (`startup_localization_bootstrap_*`) runs from idle:
      - global relocalization + nomotion update + slow in-place spin
      - periodic readiness re-check before first stable GOTO
    - optional fixed startup pose (`startup_initial_pose_*`) can publish one-shot
      `/{robot_ns}/initialpose` when the robot always starts from the same known map pose.
    - when blocked, emits `LOCALIZATION_NOT_READY` event with `reason`, `reason_code`, and `operator_hint`.
    - on `localization_not_ready`, recovery cycle can call global relocalization service + in-place spin.
    - if lifecycle is inactive, recovery also requests lifecycle manager `STARTUP`/`RESUME`.
    - on `action_server_not_ready` / `goal_rejected`, retries with delay and bounded attempts.
    - controlled by `nav2_retry_*`, `localization_*`, `localization_recovery_*`, `amcl_*` parameters.

## Bringup Arguments (Current)
- `robot_ns`
- `robot_id`
- `enable_rosbridge`
- `use_nav2`
- `nav2_action_name`
- `goal_response_timeout_sec` (default `8.0`)
- `mock_mode` (default `false`)
- `obstacle_enabled` (default `true`)
- `obstacle_topic` (default `obstacles`)
- `obstacle_presence_stop_classes` (default `person,robot`)
- `safety_state_topic` (default `safety_state`)
- `nav2_retry_attempts` (default `8`)
- `nav2_retry_delay_sec` (default `1.0`)
- `localization_required` (default `true`)
- `amcl_pose_topic` (default `amcl_pose`)
- `localization_recovery_enabled` (default `true`)
- `localization_recovery_max_cycles` (default `2`)
- `startup_initial_pose_enabled` (default `false`)
- `startup_initial_pose_topic` (default `initialpose`)
- `startup_initial_pose_x/y/yaw` (default `0.0`)

## Standard Run
```bash
cd /home/changpc/ros-repo-1/robot/jazzy_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch office_robot_bringup bringup.launch.py \
  robot_ns:=robot01 robot_id:=1 enable_rosbridge:=true \
  use_nav2:=true nav2_action_name:=/robot01/navigate_to_pose
```

## Smoke Commands
```bash
# ACTION_SEQUENCE (example)
ros2 topic pub --once /robot01/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot01\",\"type\":\"ACTION_SEQUENCE\",\"task_id\":101,\"payload\":[{\"action\":\"DISPLAY_TEXT\",\"params\":{\"text\":\"hello\"},\"on_success\":\"DONE\"}]}"}'

# STOP / RESUME
ros2 topic pub --once /robot01/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot01\",\"type\":\"STOP\",\"task_id\":102}"}'

ros2 topic pub --once /robot01/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot01\",\"type\":\"RESUME\",\"task_id\":102}"}'
```

## Ops Notes
- rosbridge port: `9090/tcp`
- UDP camera stream target: `54321/udp`
- Camera source mode:
  - `CAMERA_SOURCE=topic`: `robot-camera.service` -> `/camera/image_raw`
  - `CAMERA_SOURCE=rpicam`: `robot-udp-bridge.service` captures directly via `rpicam-vid`
  - PinkyPro default is `rpicam` (`/etc/robot_runtime.env`)
- AI link health:
  - `communication_node` publishes `/{robot_ns}/ai_link` (`true`/`false`)
  - `office_robot_executor` mirrors this as `ai_link_alive` key in `/{robot_ns}/status`
- Dynamic obstacle handling:
  - `SR-003` static obstacle avoidance remains Nav2 costmap/controller behavior.
  - `SR-004 v1` is `safe stop / resume` for dynamic `person` / `robot`.
  - Upstream bbox (`box.x/y/width/height`) is preserved into `safety_state` and `/status`.
  - Full distance-aware yield can be layered later if upstream starts sending `distance_m`
    or if robot-side box + LiDAR fusion is added.
- Localization:
  - If `amcl_pose_missing` persists after global relocalization + spin, the operator must set
    `2D Pose Estimate` once in RViz unless fixed startup pose is enabled.
  - `AMCL cannot publish a pose or update the transform. Please set the initial pose...`
    means `map->odom` will stay missing and RViz `Global Status` will remain `Error`.
- Video stream tuning defaults (battery/network friendly):
  - `max_fps=8.0`, `resize_width=640`, `resize_height=360`, `jpeg_quality=70`
- If AI vision is down:
  - stream is paused when `skip_stream_when_ai_dead=true`
  - periodic healthcheck uses `ai_healthcheck_port` (default `50052`)
- Local-only directories are ignored:
  - `robot/jazzy_ws/build`
  - `robot/jazzy_ws/install`
  - `robot/jazzy_ws/log`
  - `robot/jazzy_ws/mujoco_menagerie`

## Nav2 Runtime Recovery Procedure
```bash
# 1) Baseline audit
/home/pinky/ros-repo-1/robot/scripts/nav2_runtime_audit.sh

# 2) Install/update systemd override (root)
sudo /home/pinky/ros-repo-1/robot/scripts/install_pinky_navigation_override.sh

# 3) Verify env keys
grep -E '^(MAP_PATH|NAV2_PARAMS_FILE)=' /etc/robot_runtime.env

# 4) Restart + re-audit
sudo systemctl restart pinky-navigation.service
/home/pinky/ros-repo-1/robot/scripts/nav2_runtime_audit.sh
```

- Pass expectation from audit:
  - `local rolling_window=true`
  - `local/global observation_sources` includes `scan`
  - frame pair is `odom` (local) / `map` (global)
- Warning means service/node/parameter lookup failed.
- Fail means runtime Nav2 config is mismatched and should be fixed before E2E.
- Operational rule: do not run `navigation_launch.xml` alone in production path.
  Use `bringup_launch.xml` with explicit `params_file` chain.

## Runtime Templates
- `robot/systemd/robot-camera.service`
- `robot/systemd/robot-udp-bridge.service`
- `robot/systemd/robot_runtime.env.example`
- `robot/systemd/pinky-navigation.override.conf.example`
- `robot/scripts/camera_probe.sh`
- `robot/scripts/nav2_runtime_audit.sh`
- `robot/scripts/install_pinky_navigation_override.sh`
- `robot/scripts/run_rviz_nav_debug.sh`

## RViz Debug (On-PC)
```bash
# PC must join same network and ROS_DOMAIN_ID as robot
cd /home/changpc/ros-repo-1
ROBOT_NS=robot01 ROS_DOMAIN_ID=88 ./robot/scripts/run_rviz_nav_debug.sh

# one-command quick test
./robot/scripts/test_rviz_debug.sh
```

- RViz `Debug Overlay` display uses `/{robot_ns}/debug_markers`
  and shows latest `status` + `event` text in the scene.
