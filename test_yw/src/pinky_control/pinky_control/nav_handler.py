import math
import time
from typing import Any, Dict, List, Optional, Tuple, Callable

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:
    from nav2_msgs.srv import ManageLifecycleNodes
except ImportError:
    ManageLifecycleNodes = None

try:
    from lifecycle_msgs.srv import GetState
except ImportError:
    GetState = None

try:
    from std_srvs.srv import Empty
except ImportError:
    Empty = None

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:
    Buffer = None
    TransformListener = None


class NavHandler:
    """
    Advanced Navigation Handler for Pinky.
    Added detailed logging for debugging movement issues.
    """

    def __init__(self, node: Node, config: Dict[str, Any]):
        self.node = node
        self.p = config
        self.logger = node.get_logger()

        # Action Client
        self.nav_client = None
        if self.p['use_nav2'] and not self.p['mock_mode'] and NavigateToPose:
            self.nav_client = ActionClient(self.node, NavigateToPose, self.p['nav2_action_name'])

        # TF2 Setup
        self.tf_buffer = None
        if Buffer and TransformListener:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        # Service Clients
        self.global_loc_client = self.nomotion_client = self.lifecycle_mgr_client = None
        if Empty:
            self.global_loc_client = self.node.create_client(Empty, self.p['global_localization_service_name'])
            self.nomotion_client = self.node.create_client(Empty, self.p['amcl_nomotion_update_service_name'])
        if ManageLifecycleNodes:
            self.lifecycle_mgr_client = self.node.create_client(ManageLifecycleNodes, self.p['nav2_lifecycle_manager_service_name'])

        # Lifecycle Monitoring
        self._lifecycle_clients: Dict[str, Any] = {}
        self._lifecycle_states: Dict[str, Tuple[int, str, float]] = {}
        self._lifecycle_pending: Dict[str, bool] = {}
        
        if self.p['nav2_lifecycle_check_enabled'] and GetState:
            self.node.create_timer(1.0, self._poll_lifecycle_states)

        # Internal State
        self._current_goal_handle = None
        self._nav_callback: Optional[Callable] = None
        self._nav_retry_count = 0
        self._recovery_cycle_count = 0
        
        self.last_amcl_pose_mono = 0.0
        self.last_amcl_cov_xy = self.last_amcl_cov_yaw = 0.0
        self.last_feedback: Optional[Dict[str, Any]] = None

    def _call_later(self, delay_sec: float, callback: Callable):
        timer = None
        def _wrapper():
            nonlocal timer
            if timer:
                timer.cancel()
                self.node.destroy_timer(timer)
            callback()
        timer = self.node.create_timer(delay_sec, _wrapper)

    def _poll_lifecycle_states(self):
        nodes = [n.strip() for n in self.p['nav2_required_active_nodes'].split(',') if n.strip()]
        for node_name in nodes:
            full_name = self._resolve_node_name(node_name)
            if self._lifecycle_pending.get(full_name, False): continue
            client = self._lifecycle_clients.get(full_name)
            if not client:
                client = self.node.create_client(GetState, f"{full_name}/get_state")
                self._lifecycle_clients[full_name] = client
            if not client.wait_for_service(timeout_sec=0.1): continue
            try:
                self._lifecycle_pending[full_name] = True
                future = client.call_async(GetState.Request())
                future.add_done_callback(lambda f, fn=full_name: self._on_lifecycle_response(fn, f))
            except Exception: self._lifecycle_pending[full_name] = False

    def _on_lifecycle_response(self, full_name: str, future: Any):
        self._lifecycle_pending[full_name] = False
        try:
            res = future.result()
            self._lifecycle_states[full_name] = (res.current_state.id, res.current_state.label, time.monotonic())
        except Exception: pass

    def is_nav2_lifecycle_ready(self) -> Tuple[bool, str]:
        if not self.p['nav2_lifecycle_check_enabled']: return True, "disabled"
        nodes = [n.strip() for n in self.p['nav2_required_active_nodes'].split(',') if n.strip()]
        for node_name in nodes:
            full_name = self._resolve_node_name(node_name)
            state = self._lifecycle_states.get(full_name)
            if not state: return False, f"node_state_unknown:{node_name}"
            state_id, state_label, stamp = state
            if time.monotonic() - stamp > 5.0: return False, f"node_state_stale:{node_name}"
            if state_id != 3: return False, f"node_not_active:{node_name}:{state_label}"
        return True, "ready"

    def update_amcl_status(self, mono_time: float, cov_xy: float, cov_yaw: float):
        self.last_amcl_pose_mono = mono_time
        self.last_amcl_cov_xy, self.last_amcl_cov_yaw = cov_xy, cov_yaw

    def is_localization_ready(self) -> Tuple[bool, str]:
        if not self.p['localization_required']: return True, "disabled"
        
        if self.last_amcl_pose_mono <= 0.0: 
            self.logger.debug("NavHandler: No data received on amcl_pose yet.")
            return False, "amcl_pose_missing"
        
        age = time.monotonic() - self.last_amcl_pose_mono
        if self.p['amcl_pose_stale_check_enabled'] and age > self.p['amcl_pose_max_age_sec']:
            return False, f"amcl_pose_stale:{age:.2f}s"
        if self.last_amcl_cov_xy > self.p['amcl_covariance_xy_max']: return False, f"high_cov_xy:{self.last_amcl_cov_xy:.3f}"
        if self.last_amcl_cov_yaw > self.p['amcl_covariance_yaw_max']: return False, f"high_cov_yaw:{self.last_amcl_cov_yaw:.3f}"
        if self.p['nav2_require_map_odom_tf'] and self.tf_buffer:
            try:
                self.tf_buffer.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=self.p['nav2_tf_lookup_timeout_sec']))
            except Exception as e: return False, f"map_odom_tf_missing:{e}"
        ready, reason = self.is_nav2_lifecycle_ready()
        if not ready: return False, reason
        return True, "ready"

    def _resolve_node_name(self, name: str) -> str:
        if name.startswith("/"): return name
        ns = self.node.get_namespace().rstrip("/")
        return f"{ns}/{name}"

    def send_goto(self, x: float, y: float, yaw: float, callback: Callable):
        self._nav_callback = callback
        self.last_feedback = None
        self.logger.info(f"NavHandler: Starting GOTO to ({x}, {y}, {yaw})")

        if self.p['mock_mode']:
            self.logger.info("NavHandler: Mock mode enabled, success in 1s.")
            self._call_later(1.0, lambda: self._finish(True, "Mock Success"))
            return
        if not self.nav_client:
            self.logger.error("NavHandler: nav_client is NULL")
            self._finish(False, "nav_client_unavailable"); return

        ready, reason = self.is_localization_ready()
        if not ready:
            self.logger.warn(f"NavHandler: Localization not ready: {reason}")
            if self._nav_retry_count < self.p['nav2_retry_attempts']:
                self._nav_retry_count += 1
                self.logger.info(f"NavHandler: Scheduling retry {self._nav_retry_count}/{self.p['nav2_retry_attempts']} in {self.p['nav2_retry_delay_sec']}s")
                self._trigger_recovery(reason)
                self._call_later(self.p['nav2_retry_delay_sec'], lambda: self.send_goto(x, y, yaw, callback))
                return
            else:
                self.logger.error(f"NavHandler: Max retries reached. Failing.")
                self._finish(False, f"localization_not_ready:{reason}"); return

        self.logger.info("NavHandler: Waiting for Nav2 action server...")
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.logger.error("NavHandler: Nav2 server timeout!")
            self._finish(False, "nav2_server_timeout"); return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.p['frame_id']
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
        
        self.logger.info("NavHandler: Sending goal request to Nav2...")
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.last_feedback = {"distance_remaining": fb.distance_remaining, "current_x": fb.current_pose.pose.position.x, "current_y": fb.current_pose.pose.position.y}

    def _on_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.logger.warn("NavHandler: Goal REJECTED by Nav2 server")
                self._finish(False, "goal_rejected"); return
            self._current_goal_handle = handle
            self.logger.info("NavHandler: Goal ACCEPTED. Tracking execution...")
            handle.get_result_async().add_done_callback(self._on_result)
        except Exception as e:
            self.logger.error(f"NavHandler: Goal response exception: {e}")
            self._finish(False, f"send_goal_exception:{e}")

    def _on_result(self, future):
        try:
            result = future.result()
            self.logger.info(f"NavHandler: Goal finished with status {result.status}")
            if result.status == self.p['nav2_success_status_code']:
                self._nav_retry_count = self._recovery_cycle_count = 0
                self._finish(True, "success")
            elif self._check_abort_as_success(result.status):
                self.logger.warn("NavHandler: Goal aborted but close enough. Treating as success.")
                self._finish(True, "aborted_but_close_enough")
            else:
                self.logger.error(f"NavHandler: Goal failed with status {result.status}")
                self._finish(False, f"nav2_failed_status:{result.status}")
        except Exception as e:
            self.logger.error(f"NavHandler: Result exception: {e}")
            self._finish(False, f"result_exception:{e}")

    def _check_abort_as_success(self, status: int) -> bool:
        if not self.p['nav2_abort_as_success_enabled'] or status != 6: return False
        if not self.last_feedback: return False
        return self.last_feedback.get("distance_remaining", 999.0) <= self.p['nav2_abort_success_distance_tolerance']

    def cancel_goal(self):
        if self._current_goal_handle:
            self.logger.info("NavHandler: Cancelling active goal.")
            self._current_goal_handle.cancel_goal_async()
        self._current_goal_handle = None

    def _trigger_recovery(self, reason: str):
        if not self.p['localization_recovery_enabled'] or self._recovery_cycle_count >= self.p['localization_recovery_max_cycles']: return
        self._recovery_cycle_count += 1
        self.logger.warn(f"NavHandler: Triggering recovery cycle {self._recovery_cycle_count} for {reason}")
        if "amcl" in reason or "high" in reason:
            if self.global_loc_client: self.global_loc_client.call_async(Empty.Request())
        if "node_not_active" in reason and self.lifecycle_mgr_client:
            req = ManageLifecycleNodes.Request(); req.command = 0; self.lifecycle_mgr_client.call_async(req)
        if self.nomotion_client: self.nomotion_client.call_async(Empty.Request())
        if hasattr(self.node, '_start_localization_spin'): self.node._start_localization_spin()

    def _finish(self, success: bool, msg: str, feedback: Optional[Dict] = None):
        self._current_goal_handle = None
        if self._nav_callback:
            cb = self._nav_callback; self._nav_callback = None
            cb(success, msg, feedback)
