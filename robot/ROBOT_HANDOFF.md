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

## Namespace / Multi-Robot
- Default namespace: `robot_1`
- Launch arg: `robot_ns` (example: `robot_1`, `robot_2`)
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

## Bringup Arguments (Current)
- `robot_ns`
- `robot_id`
- `enable_rosbridge`
- `use_nav2`
- `nav2_action_name`
- `mock_mode` (default `false`)

## Standard Run
```bash
cd /home/changpc/ros-repo-1/robot/jazzy_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch office_robot_bringup bringup.launch.py \
  robot_ns:=robot_1 robot_id:=1 enable_rosbridge:=true \
  use_nav2:=true nav2_action_name:=/robot_1/navigate_to_pose
```

## Smoke Commands
```bash
# ACTION_SEQUENCE (example)
ros2 topic pub --once /robot_1/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot_1\",\"type\":\"ACTION_SEQUENCE\",\"task_id\":101,\"payload\":[{\"action\":\"DISPLAY_TEXT\",\"params\":{\"text\":\"hello\"},\"on_success\":\"DONE\"}]}"}'

# STOP / RESUME
ros2 topic pub --once /robot_1/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot_1\",\"type\":\"STOP\",\"task_id\":102}"}'

ros2 topic pub --once /robot_1/commands std_msgs/msg/String \
'{data: "{\"robot_name\":\"robot_1\",\"type\":\"RESUME\",\"task_id\":102}"}'
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

## Runtime Templates
- `robot/systemd/robot-camera.service`
- `robot/systemd/robot-udp-bridge.service`
- `robot/systemd/robot_runtime.env.example`
- `robot/scripts/camera_probe.sh`
