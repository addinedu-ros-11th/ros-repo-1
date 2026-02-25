# Robot Folder Structure

## Core Tree (Current)
```text
robot/
  README.md
  ROBOT_HANDOFF.md
  INTEGRATION_GUIDE.md
  docs/
    STRUCTURE.md
    pinky_navigation_cmdvel_remap.patch
  jazzy_ws/
    src/
      communication_node/
      office_robot_bridge/
      office_robot_bringup/
      office_robot_executor/
      office_robot_safety/
    build/                  # local build artifacts
    install/                # local install artifacts
    log/                    # local runtime/build logs
    mujoco_menagerie/       # local experimental folder (ignored)
```

## What To Edit
- Runtime logic: `robot/jazzy_ws/src/**`
- Launch/config wiring: `office_robot_bringup/launch`, `office_robot_bringup/config`
- Robot docs: `robot/*.md`, `robot/docs/*.md`

## What Not To Commit
- `robot/jazzy_ws/build`
- `robot/jazzy_ws/install`
- `robot/jazzy_ws/log`
- `robot/jazzy_ws/mujoco_menagerie`

## Quick Checks
```bash
git status -sb
cd robot/jazzy_ws && colcon build --symlink-install
```
