# Robot Integration Guide

This document describes how the robot stack connects to the main server and the AI server, plus the recommended integration patterns and operational checks.

## Scope

- Main server: the primary backend that owns user requests, task orchestration, and persistent data.
- AI server: the service that provides inference, planning, or natural language processing.
- Robot stack: ROS 2 nodes inside `robot-han/jazzy_ws`.

## High-Level Architecture

- The robot stack publishes telemetry and receives commands over a well-defined API surface.
- The main server is the source of truth for task state and should assign tasks to the robot.
- The AI server provides decisions or enriched responses; it should not directly control actuators.

Recommended data flow:

1. Robot -> Main server: telemetry, status, task progress.
2. Main server -> Robot: commands, task assignments, configuration updates.
3. Main server <-> AI server: context, prompts, inference results.

## Integration Options

### 1) REST/HTTP API (Recommended for control + telemetry)

Use this when your main server is a standard web backend.

Robot -> Main server
- `POST /api/robot/telemetry` with battery, pose, system health, and current task status.
- `POST /api/robot/events` for discrete events (obstacle detected, task finished).

Main server -> Robot
- `GET /api/robot/commands?robot_id=...` for command polling, or
- `POST /api/robot/command` to push commands if the robot exposes an endpoint.

Notes
- Use idempotent command IDs.
- If you must poll, keep a low cadence (e.g., 1-2 Hz) and implement backoff.

### 2) WebSocket (Recommended for near-real-time control)

Use this when you need low-latency command and telemetry streams.

- Single persistent connection between robot and main server.
- Messages are JSON with a `type` field (e.g., `telemetry`, `command`, `ack`).
- Include `robot_id`, `command_id`, and `timestamp` in every message.

### 3) gRPC (Recommended for structured, typed APIs)

Use this if your main server and robot stack are both comfortable with protobufs.

- Define `RobotTelemetry`, `RobotCommand`, `CommandAck` in a `.proto`.
- Enables strong typing, streaming, and automatic client generation.

### 4) ROS 2 Bridge (For server-side ROS integration)

Use this if the main server runs in the same ROS 2 domain or uses `rosbridge`.

- Robot nodes publish to ROS 2 topics.
- Server-side nodes subscribe and push commands to robot topics.

Caution
- Validate security carefully if the ROS network crosses machines or subnets.

## Main Server Integration

### Responsibilities

- Assign tasks and track task state.
- Persist telemetry and logs.
- Enforce robot safety rules (speed, area constraints).

### Suggested endpoints or message types

- `telemetry`: battery, pose, velocity, map frame, CPU/RAM usage.
- `task_update`: task_id, progress, current step, ETA.
- `command`: command_id, action, params, priority.
- `ack`: command_id, status (`accepted`, `rejected`, `done`, `failed`).

## AI Server Integration

### Responsibilities

- Provide planning, classification, or natural language inference.
- Return structured results rather than raw prose whenever possible.

### Suggested interaction pattern

- Main server sends a context payload to AI server.
- AI server returns a structured plan (JSON) with confidence.
- Main server validates and translates that plan into robot commands.

Example AI response format

```json
{
  "intent": "deliver_item",
  "confidence": 0.86,
  "plan": [
    {"step": "navigate", "target": "Room 310"},
    {"step": "notify_user", "message": "Arrived"}
  ]
}
```

## Reliability and Safety

- Always treat the main server as the single authority for task state.
- Add timeouts and retries with exponential backoff for all network calls.
- Use command acknowledgements and a dead-letter queue for failed commands.
- Ensure the robot can safely stop if connectivity is lost.

## Security

- Use TLS for all external connections.
- Authenticate robot clients with short-lived tokens.
- Rotate credentials and log all control commands.

## Testing Checklist

- Telemetry flow: robot -> main server
- Command flow: main server -> robot
- AI flow: main server <-> AI server
- Failover: network loss, server restart, retry behavior
- Safety: emergency stop and safe idle state

## Next Steps

- Decide on one primary integration method (REST, WebSocket, gRPC, ROS 2 bridge).
- Document concrete endpoints and message schemas.
- Add a small mock server to test command and telemetry flow locally.
