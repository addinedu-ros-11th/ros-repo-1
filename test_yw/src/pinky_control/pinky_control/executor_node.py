import json
import math
import time
from typing import Any, Dict, List, Optional
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

# Import Handlers
from pinky_control.qr_handler import QRHandler
from pinky_control.nav_handler import NavHandler


class PinkyExecutor(Node):
    """
    Enhanced Pinky Executor.
    Optimized for TaskProcessors and ROS Bridge compatibility.
    Includes robust error handling to prevent crashes.
    """

    def __init__(self) -> None:
        super().__init__("executor_node")

        # --- Parameter Declaration ---
        self._declare_params()
        self._load_params()

        # --- Sub-Handlers ---
        self.qr_handler = QRHandler()
        self.nav_handler = NavHandler(self, self.p)

        # --- Internal State ---
        self.location = (0.0, 0.0)
        self.current_status = "IDLE"
        self.battery = self.p.get('initial_battery', 100.0)
        
        self._current_task_id = None
        self._action_queue: List[Dict[str, Any]] = []
        self._current_action: Optional[Dict[str, Any]] = None
        self._action_timer = None
        
        self._safety_locked = False
        self._ai_link_alive: Optional[bool] = None
        self._latest_qr_image: Optional[bytes] = None
        
        # Display/LED State
        self._guide_display_timer = None
        self._guide_display_toggle = False
        self._current_led = {"color": "OFF", "mode": "SOLID"}

        # --- ROS Subscriptions ---
        self.command_sub = self.create_subscription(String, "commands", self._on_commands, 10)
        self.odom_sub = self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 10)
        self.battery_sub = self.create_subscription(Float32, self.p['battery_topic'], self._on_battery, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.p['amcl_pose_topic'], self._on_amcl_pose, 10)

        safety_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.safety_sub = self.create_subscription(Bool, self.p['safety_lock_topic'], self._on_safety_lock, safety_qos)
        self.ai_link_sub = self.create_subscription(Bool, self.p['ai_link_topic'], self._on_ai_link, safety_qos)

        if self.p['qr_scan_local_enabled'] or self.p['qr_always_scan_enabled']:
            self.qr_sub = self.create_subscription(CompressedImage, self.p['qr_scan_image_topic'], self._on_qr_image, 10)

        # --- ROS Publishers ---
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)
        self.display_pub = self.create_publisher(String, self.p['display_topic'], 10)
        self.stop_pub = self.create_publisher(Twist, self.p['stop_cmd_vel_topic'], 10)

        # --- Timers ---
        self.status_timer = self.create_timer(2.0, self._publish_heartbeat)
        
        if self.p['qr_always_scan_enabled']:
            self.create_timer(self.p['qr_always_scan_poll_period_sec'], self._poll_always_qr_scan)

        self.get_logger().info(f"Pinky Executor {self.robot_name} (ID:{self.robot_id}) ready.")
        self._publish_display("준비 완료", "check")

    def _declare_params(self):
        params = [
            ("robot_name", "pinky"), ("robot_id", 1), ("mock_mode", False),
            ("use_nav2", True), ("execution_delay_sec", 1.5), ("initial_battery", 100.0),
            ("battery_topic", "battery/present"), ("nav2_action_name", "navigate_to_pose"),
            ("frame_id", "map"), ("goal_timeout_sec", 120.0), ("goal_response_timeout_sec", 10.0),
            ("stop_cmd_vel_topic", "/cmd_vel"), ("stop_publish_count", 10), ("stop_publish_hz", 20.0),
            ("safety_lock_topic", "safety_lock"), ("ai_link_topic", "ai_link"), ("include_ai_link_in_status", True),
            ("nav2_success_status_code", 4), ("nav2_feedback_log_period_sec", 1.5),
            ("nav2_abort_as_success_enabled", False), ("nav2_abort_success_distance_tolerance", 0.35),
            ("nav2_retry_attempts", 3), ("nav2_retry_delay_sec", 2.0),
            ("localization_required", True), ("amcl_pose_topic", "amcl_pose"), ("odom_topic", "odom"),
            ("amcl_pose_max_age_sec", 5.0), ("amcl_pose_stale_check_enabled", True),
            ("amcl_covariance_xy_max", 0.5), ("amcl_covariance_yaw_max", 6.0),
            ("localization_allow_degraded_covariance", True), ("nav2_require_map_odom_tf", True),
            ("nav2_tf_lookup_timeout_sec", 0.1), ("localization_recovery_enabled", True),
            ("localization_recovery_max_cycles", 2), ("localization_recovery_spin_duration_sec", 4.0),
            ("localization_recovery_spin_angular_speed", 0.8), ("display_topic", "display"),
            ("qr_scan_local_enabled", True), ("qr_scan_image_topic", "camera/image_raw/compressed"),
            ("qr_scan_timeout_sec", 15.0), ("qr_scan_poll_period_sec", 0.2),
            ("qr_always_scan_enabled", True), ("qr_always_scan_poll_period_sec", 1.0),
            ("guide_display_period_sec", 2.0), ("enable_display", True),
            ("qr_always_scan_event_name", "QR_DETECTED"), ("qr_always_scan_min_interval_sec", 3.0),
            ("emit_command_received_event", True), ("default_goto_success_event", "ARRIVED_AT_DESTINATION"),
            ("global_localization_service_name", "reinitialize_global_localization"),
            ("amcl_nomotion_update_service_name", "request_nomotion_update"),
            ("nav2_lifecycle_check_enabled", True), ("nav2_lifecycle_reactivate_enabled", True),
            ("nav2_lifecycle_manager_service_name", "lifecycle_manager_navigation/manage_nodes"),
            ("nav2_required_active_nodes", "planner_server,controller_server,bt_navigator,behavior_server"),
            ("nav2_lifecycle_get_state_timeout_sec", 0.2), ("nav2_lifecycle_manager_wait_sec", 1.0)
        ]
        for name, default in params:
            self.declare_parameter(name, default)

    def _load_params(self):
        self.p = {name: self.get_parameter(name).value for name in self._parameters}
        self.robot_id = self.p['robot_id']
        self.robot_name = self.p['robot_name']

    def _publish_status(self, status: str, event: Optional[str] = None, extra: Optional[Dict] = None):
        try:
            data = {
                "robot_id": self.robot_id, "robot_name": self.robot_name, "status": status,
                "location": [round(float(self.location[0]), 3), round(float(self.location[1]), 3)],
                "battery": round(float(self.battery), 1), "safety_lock": self._safety_locked,
                "task_id": self._current_task_id
            }
            if self.p['include_ai_link_in_status']: data["ai_link"] = self._ai_link_alive
            if event: data["event"] = event
            if extra: data.update(extra)
            
            # ensure_ascii=False for Korean support, default=str for safety
            self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False, default=str)))
        except Exception as e:
            self.get_logger().error(f"Status publish failed: {e}")

    def _publish_event(self, event: str, extra: Dict):
        try:
            data = {"event": event, "robot_id": self.robot_id, "robot_name": self.robot_name, "ts": time.time()}
            data.update(extra)
            self.event_pub.publish(String(data=json.dumps(data, ensure_ascii=False, default=str)))
        except Exception:
            pass

    def _publish_heartbeat(self):
        try:
            self._publish_status(self.current_status)
        except Exception:
            pass

    def _on_commands(self, msg: String):
        try: 
            payload = json.loads(msg.data)
        except: 
            return
        
        if not self._is_for_this_robot(payload): return
        
        try:
            cmd_type = str(payload.get("type", "")).upper().strip()
            actions = self._extract_actions(payload)
            
            if self.p['emit_command_received_event']:
                self._publish_event("COMMAND_RECEIVED", {"type": cmd_type, "action_count": len(actions)})

            if cmd_type in ["STOP", "PAUSE"]:
                self._set_safety_lock(True, f"command:{cmd_type}"); return
            if cmd_type == "RESUME":
                self._set_safety_lock(False, "command:RESUME"); return
            if cmd_type == "CANCEL":
                self._cancel_active_sequence("User Cancel"); return

            if not actions: return
            if self._safety_locked:
                self._publish_status(self.current_status, event="ACTION_FAILED", extra={"reason": "safety_locked"})
                return
            
            if self._action_queue or self._current_action:
                self.get_logger().warn("Executor busy, rejecting sequence.")
                return

            self._action_queue = actions
            self._current_task_id = payload.get("task_id", payload.get("sequence_id"))
            self._publish_status("ASSIGNED")
            self._run_next_action()
        except Exception as e:
            self.get_logger().error(f"Command processing error: {e}")
            traceback.print_exc()

    def _set_safety_lock(self, enabled: bool, source: str):
        if self._safety_locked == enabled: return
        self._safety_locked = enabled
        if self._safety_locked:
            self.get_logger().warn(f"Safety Lock Engaged (Source: {source})")
            self._publish_display("일시정지", "pause")
            if self.current_status in ["MOVING", "GUIDING", "WAITING"]:
                self._cancel_active_sequence(f"Safety Lock ({source})")
            self._publish_status("WAITING", event="SAFETY_STOPPED")
        else:
            self.get_logger().info(f"Safety Lock Released (Source: {source})")
            self._publish_display("대기", "idle")
            self._publish_status("IDLE", event="SAFETY_RESUMED")

    def _on_safety_lock(self, msg: Bool): self._set_safety_lock(msg.data, "topic")
    def _on_ai_link(self, msg: Bool): self._ai_link_alive = msg.data
    def _on_battery(self, msg: Float32): self.battery = msg.data

    def _on_odom(self, msg: Odometry):
        self.location = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        self.nav_handler.update_amcl_status(time.monotonic(), max(cov[0], cov[7]), cov[35])

    def _on_qr_image(self, msg: CompressedImage):
        self._latest_qr_image = bytes(msg.data)

    def _run_next_action(self):
        try:
            if self._action_timer: self._action_timer.cancel(); self._action_timer = None
                
            if not self._action_queue:
                self.current_status = "IDLE"; self._current_task_id = self._current_action = None
                self._publish_display("대기", "idle"); self._stop_guide_display(); self._publish_status("IDLE")
                return

            action_data = self._action_queue.pop(0)
            self._current_action = action_data
            action = str(action_data.get("action", action_data.get("type", ""))).upper().strip()
            params = action_data.get("params", {})
            on_success = action_data.get("on_success")

            self.get_logger().info(f"Task {self._current_task_id}: Executing {action}")

            if action in ["GOTO", "LEAD_GUEST"]:
                self.current_status = "GUIDING" if action == "LEAD_GUEST" else "MOVING"
                self._publish_display("이동 중" if action == "GOTO" else "안내 중", "delivery" if action == "GOTO" else "guide")
                if action == "LEAD_GUEST": self._start_guide_display()
                self._publish_status(self.current_status)
                self.nav_handler.send_goto(float(params.get("x", 0.0)), float(params.get("y", 0.0)), float(params.get("yaw", 0.0)), self._on_goto_finished)
                
            elif action == "QR_SCAN":
                self.current_status = "WAITING"; self._publish_display("QR 스캔 중", "qr"); self._publish_status("WAITING")
                self._start_qr_scan_loop(on_success)
                
            elif action == "DISPLAY_TEXT":
                text = params.get("text", "안내중")
                duration = float(params.get("duration", self.p['execution_delay_sec']))
                self._publish_display(text, "display"); self._publish_status("WAITING")
                if duration > 0: self._action_timer = self.create_timer(duration, self._run_next_action)
                
            elif action == "SET_LED":
                self._current_led = {"color": params.get("color", "OFF"), "mode": params.get("mode", "SOLID")}
                self.get_logger().info(f"LED Changed: {self._current_led}")
                self._run_next_action()
                
            elif action == "WAIT":
                self.current_status = "WAITING"; self._publish_status("WAITING")
                self._action_timer = self.create_timer(float(params.get("duration", 2.0)), self._run_next_action)
                
            elif action == "CANCEL":
                self._cancel_active_sequence("Action Request")
            else:
                self.get_logger().error(f"Unknown action: {action}")
                self._run_next_action()
        except Exception as e:
            self.get_logger().error(f"Action execution error: {e}")
            traceback.print_exc()
            self._cancel_active_sequence(f"Error: {e}")

    def _on_goto_finished(self, success: bool, msg: str, feedback: Optional[Dict]):
        if success:
            event = self._current_action.get("on_success", self.p['default_goto_success_event'])
            self._publish_status(self.current_status, event=event)
            self._run_next_action()
        else:
            self._publish_status("ERROR", event="ACTION_FAILED", extra={"reason": msg})
            self._cancel_active_sequence(msg)

    def _start_qr_scan_loop(self, on_success: Optional[str]):
        start_time = time.monotonic()
        def _poll():
            if self._safety_locked or self.current_status != "WAITING": return
            if time.monotonic() - start_time > self.p['qr_scan_timeout_sec']:
                self._publish_status("WAITING", event="ACTION_FAILED", extra={"reason": "qr_timeout"})
                self._run_next_action(); return
            
            decoded = self.qr_handler.decode(self._latest_qr_image)
            if decoded:
                event = on_success or "QR_SCANNED"
                self._publish_status(self.current_status, event=event, extra={"scanned_data": decoded})
                self._run_next_action(); return
            
            # Manually handle one-shot by creating a new timer for each poll
            self._action_timer = self.create_timer(self.p['qr_scan_poll_period_sec'], _poll)
        _poll()

    def _poll_always_qr_scan(self):
        if self.current_status == "WAITING": return
        decoded = self.qr_handler.decode(self._latest_qr_image)
        if decoded: self._publish_event(self.p['qr_always_scan_event_name'], {"scanned_data": decoded})

    def _cancel_active_sequence(self, reason: str):
        self._action_queue = []; self._current_action = None
        if self._action_timer: self._action_timer.cancel()
        self.nav_handler.cancel_goal(); self._stop_guide_display(); self._publish_zero_cmd_vel()
        self.current_status = "IDLE"; self._publish_status("IDLE", event="SEQUENCE_CANCELED", extra={"reason": reason})
        self._current_task_id = None

    def _publish_zero_cmd_vel(self):
        msg = Twist()
        for _ in range(self.p['stop_publish_count']):
            self.stop_pub.publish(msg); time.sleep(1.0 / self.p['stop_publish_hz'])

    def _is_for_this_robot(self, payload: Dict) -> bool:
        t_id, t_name = payload.get("robot_id"), payload.get("robot_name")
        if t_id is not None and int(t_id) != self.robot_id: return False
        if t_name is not None and t_name != self.robot_name: return False
        return True

    def _extract_actions(self, payload: Dict) -> List[Dict]:
        if "actions" in payload: return payload["actions"]
        if "payload" in payload and isinstance(payload["payload"], list): return payload["payload"]
        return [payload] if "action" in payload else []

    def _publish_display(self, text: str, icon: str = "info"):
        if not self.p['enable_display']: return
        self.display_pub.publish(String(data=json.dumps({"text": text, "icon": icon, "robot_id": self.robot_id, "ts": time.time()}, ensure_ascii=False)))

    def _start_localization_spin(self):
        if not self.p['localization_recovery_enabled'] or self._safety_locked: return
        self._recovery_spin_until = time.monotonic() + self.p['localization_recovery_spin_duration_sec']
        self._recovery_spin_timer = self.create_timer(0.1, self._on_localization_spin_timer)

    def _on_localization_spin_timer(self):
        if self._safety_locked or time.monotonic() >= self._recovery_spin_until:
            if self._recovery_spin_timer: self._recovery_spin_timer.cancel(); self._recovery_spin_timer = None
            self._publish_zero_cmd_vel(); return
        twist = Twist(); twist.angular.z = float(self.p['localization_recovery_spin_angular_speed'])
        self.stop_pub.publish(twist)

    def _start_guide_display(self):
        self._stop_guide_display()
        def _tick():
            self._guide_display_toggle = not self._guide_display_toggle
            self._publish_display("안내 중" if self._guide_display_toggle else "Follow Me", "guide")
        self._guide_display_timer = self.create_timer(self.p['guide_display_period_sec'], _tick)

    def _stop_guide_display(self):
        if self._guide_display_timer: self._guide_display_timer.cancel(); self._guide_display_timer = None

def main():
    rclpy.init(); node = PinkyExecutor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
