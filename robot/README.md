# Robot Runtime (ROS 2 Jazzy)

`robot/` contains the robot-side runtime workspace and integration docs.

## Workspace
- Source workspace: `robot/jazzy_ws/src`
- Build artifacts (local only): `robot/jazzy_ws/build`, `robot/jazzy_ws/install`, `robot/jazzy_ws/log`
- Local experiments (ignored): `robot/jazzy_ws/mujoco_menagerie`

## ROS 2 Packages (Current)
- `communication_node`: camera image -> UDP stream (`54321/udp`)
- `office_robot_bridge`: rosbridge launch wrapper (`9090/tcp`)
- `office_robot_bringup`: unified launch entrypoint
- `office_robot_executor`: command execution (`commands/status/event`)
- `office_robot_safety`: STOP/PAUSE/RESUME safety lock + zero `cmd_vel` hold

## Runtime Contract (Robot Side)
- Command topic: `/{robot_ns}/commands` (`std_msgs/msg/String`, JSON payload)
- Status topic: `/{robot_ns}/status`
- Event topic: `/{robot_ns}/event`
- rosbridge: `/{robot_ns}` graph exposed via WebSocket `9090`

## Build / Run
```bash
cd robot/jazzy_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch office_robot_bringup bringup.launch.py \
  robot_ns:=robot_1 robot_id:=1 enable_rosbridge:=true
```

## Related Docs
- `robot/ROBOT_HANDOFF.md`
- `robot/INTEGRATION_GUIDE.md`
- `robot/docs/STRUCTURE.md`
