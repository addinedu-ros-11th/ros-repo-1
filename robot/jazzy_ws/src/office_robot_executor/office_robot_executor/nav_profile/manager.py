import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from .scan_analyzer import ScanSpaceSample, analyze_scan_space


@dataclass(frozen=True)
class DynamicNavProfileSettings:
    enabled: bool
    scan_topic: str
    robot_width_m: float
    wide_width_enter_m: float
    wide_width_exit_m: float
    forward_enter_m: float
    forward_exit_m: float
    enter_samples: int
    exit_samples: int
    wide_lookahead_dist: float
    wide_min_lookahead_dist: float
    wide_max_lookahead_dist: float
    wide_rotate_to_heading_min_angle: float


class DynamicNavProfileManager:
    PROFILE_BASELINE = "BASELINE"
    PROFILE_WIDE = "WIDE"
    _PARAM_SERVICE_TIMEOUT_SEC = 0.5
    _RETRY_TIMER_PERIOD_SEC = 0.5

    _PARAM_LOOKAHEAD = "FollowPath.lookahead_dist"
    _PARAM_MIN_LOOKAHEAD = "FollowPath.min_lookahead_dist"
    _PARAM_MAX_LOOKAHEAD = "FollowPath.max_lookahead_dist"
    _PARAM_ROTATE_MIN_ANGLE = "FollowPath.rotate_to_heading_min_angle"
    _PARAM_NAMES = (
        _PARAM_LOOKAHEAD,
        _PARAM_MIN_LOOKAHEAD,
        _PARAM_MAX_LOOKAHEAD,
        _PARAM_ROTATE_MIN_ANGLE,
    )

    def __init__(
        self,
        *,
        node: Any,
        settings: DynamicNavProfileSettings,
        controller_node_full_name: str,
        publish_event_cb: Callable[[str, Dict[str, Any]], None],
    ) -> None:
        self._node = node
        self._settings = settings
        self._publish_event_cb = publish_event_cb
        self._controller_node_full_name = controller_node_full_name

        self._param_client: Optional[AsyncParameterClient] = None
        self._scan_sub = None
        self._marker_pub = None
        self._retry_timer = None
        if self._settings.enabled:
            self._param_client = AsyncParameterClient(node, controller_node_full_name)
            self._scan_sub = node.create_subscription(
                LaserScan,
                self._settings.scan_topic,
                self._on_scan,
                10,
            )
            self._marker_pub = node.create_publisher(MarkerArray, "nav_profile_markers", 10)
            self._retry_timer = node.create_timer(
                self._RETRY_TIMER_PERIOD_SEC,
                self._on_retry_timer,
            )

        self._nav_active = False
        self._current_state = self.PROFILE_BASELINE
        self._baseline_params: Optional[Dict[str, float]] = None
        self._pending_get = False
        self._pending_set = False
        self._deferred_state: Optional[str] = None
        self._deferred_reason = ""
        self._enter_count = 0
        self._exit_count = 0
        self._last_sample: Optional[ScanSpaceSample] = None
        self._last_sample_mono = 0.0
        self._last_applied_params: Optional[Dict[str, float]] = None

    def set_nav_active(self, active: bool, reason: str) -> None:
        if not self._settings.enabled:
            return

        active = bool(active)
        if self._nav_active == active:
            if not active and self._current_state != self.PROFILE_BASELINE:
                self._request_state(self.PROFILE_BASELINE, reason)
            return

        self._nav_active = active
        self._enter_count = 0
        self._exit_count = 0
        if not active:
            self._request_state(self.PROFILE_BASELINE, reason)
            self._publish_deleteall_markers()
            return
        self._ensure_baseline(reason)
        self._request_state(
            self.PROFILE_BASELINE,
            f"{reason}:nav_start_sync",
            force=True,
        )

    def get_status_fields(self) -> Dict[str, Any]:
        if not self._settings.enabled:
            return {}

        data: Dict[str, Any] = {"nav_profile_state": self._current_state}
        if (
            self._last_sample is not None
            and self._last_sample_mono > 0.0
            and (time.monotonic() - self._last_sample_mono) <= 1.5
        ):
            if self._last_sample.estimated_width_m is not None:
                data["nav_profile_width_m"] = float(self._last_sample.estimated_width_m)
            if self._last_sample.forward_clear_m is not None:
                data["nav_profile_forward_clear_m"] = float(self._last_sample.forward_clear_m)
        return data

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._settings.enabled or not self._nav_active:
            return

        sample = analyze_scan_space(
            msg,
            robot_width_m=self._settings.robot_width_m,
        )
        self._last_sample = sample
        self._last_sample_mono = time.monotonic()
        self._publish_markers(msg, sample)
        if not sample.valid:
            self._enter_count = 0
            self._exit_count = 0
            return

        width_m = float(sample.estimated_width_m)
        forward_m = float(sample.forward_clear_m)
        desired_state = self._current_state

        if (
            width_m >= self._settings.wide_width_enter_m
            and forward_m >= self._settings.forward_enter_m
        ):
            self._enter_count += 1
            self._exit_count = 0
            if self._enter_count >= self._settings.enter_samples:
                desired_state = self.PROFILE_WIDE
        elif (
            width_m <= self._settings.wide_width_exit_m
            or forward_m <= self._settings.forward_exit_m
        ):
            self._exit_count += 1
            self._enter_count = 0
            if self._exit_count >= self._settings.exit_samples:
                desired_state = self.PROFILE_BASELINE
        else:
            self._enter_count = 0
            self._exit_count = 0

        if desired_state != self._current_state:
            self._request_state(desired_state, "scan_state_transition")

    def _ensure_baseline(self, reason: str) -> None:
        if not self._settings.enabled or self._param_client is None:
            return
        if self._baseline_params is not None or self._pending_get:
            return
        if not self._param_client.wait_for_services(timeout_sec=self._PARAM_SERVICE_TIMEOUT_SEC):
            self._node.get_logger().warn(
                "Dynamic nav profile baseline read skipped: parameter service unavailable "
                f"(node={self._controller_node_full_name}, reason={reason})."
            )
            return
        try:
            future = self._param_client.get_parameters(list(self._PARAM_NAMES))
            self._pending_get = True
            future.add_done_callback(
                lambda f, r=reason: self._on_baseline_result(f, r)
            )
        except Exception as exc:
            self._node.get_logger().warn(
                "Dynamic nav profile baseline request failed "
                f"(node={self._controller_node_full_name}, reason={reason}, error={exc})."
            )

    def _on_baseline_result(self, future: Any, reason: str) -> None:
        self._pending_get = False
        baseline: Dict[str, float] = {}
        try:
            result = future.result()
            values = list(getattr(result, "values", []))
            if len(values) != len(self._PARAM_NAMES):
                raise RuntimeError("baseline parameter count mismatch")
            for name, param_value in zip(self._PARAM_NAMES, values):
                value = _parameter_value_to_python(param_value)
                if not isinstance(value, (int, float)):
                    raise RuntimeError(f"baseline param {name} is not numeric")
                baseline[name] = float(value)
        except Exception as exc:
            self._node.get_logger().warn(
                "Dynamic nav profile baseline read failed "
                f"(node={self._controller_node_full_name}, reason={reason}, error={exc})."
            )
            return

        self._baseline_params = baseline
        self._last_applied_params = dict(baseline)
        self._node.get_logger().info(
            "Dynamic nav profile baseline captured "
            f"(node={self._controller_node_full_name}, baseline={baseline})."
        )

        if self._deferred_state is not None:
            deferred_state = self._deferred_state
            deferred_reason = self._deferred_reason or reason
            self._deferred_state = None
            self._deferred_reason = ""
            self._request_state(deferred_state, deferred_reason)

    def _request_state(self, target_state: str, reason: str, force: bool = False) -> None:
        if not self._settings.enabled or self._param_client is None:
            return
        target_state = self.PROFILE_WIDE if target_state == self.PROFILE_WIDE else self.PROFILE_BASELINE

        if self._pending_set:
            self._deferred_state = target_state
            self._deferred_reason = reason
            return

        if self._baseline_params is None:
            self._deferred_state = target_state
            self._deferred_reason = reason
            self._ensure_baseline(reason)
            return

        if (
            not force
            and target_state == self._current_state
            and self._last_applied_params is not None
        ):
            return

        target_params = self._target_params_for_state(target_state)
        if not force and self._last_applied_params == target_params:
            self._current_state = target_state
            return

        if not self._param_client.wait_for_services(timeout_sec=self._PARAM_SERVICE_TIMEOUT_SEC):
            self._deferred_state = target_state
            self._deferred_reason = reason
            self._node.get_logger().warn(
                "Dynamic nav profile apply skipped: parameter service unavailable "
                f"(node={self._controller_node_full_name}, target_state={target_state}, reason={reason})."
            )
            return

        try:
            params = [
                Parameter(name, value=float(value))
                for name, value in target_params.items()
            ]
            future = self._param_client.set_parameters_atomically(params)
            self._pending_set = True
            future.add_done_callback(
                lambda f, s=target_state, p=dict(target_params), r=reason: self._on_set_result(
                    f, s, p, r
                )
            )
        except Exception as exc:
            self._node.get_logger().warn(
                "Dynamic nav profile apply request failed "
                f"(node={self._controller_node_full_name}, target_state={target_state}, reason={reason}, error={exc})."
            )

    def _target_params_for_state(self, state: str) -> Dict[str, float]:
        if self._baseline_params is None:
            return {}
        if state == self.PROFILE_WIDE:
            return {
                self._PARAM_LOOKAHEAD: float(self._settings.wide_lookahead_dist),
                self._PARAM_MIN_LOOKAHEAD: float(self._settings.wide_min_lookahead_dist),
                self._PARAM_MAX_LOOKAHEAD: float(self._settings.wide_max_lookahead_dist),
                self._PARAM_ROTATE_MIN_ANGLE: float(
                    self._settings.wide_rotate_to_heading_min_angle
                ),
            }
        return dict(self._baseline_params)

    def _on_set_result(
        self,
        future: Any,
        target_state: str,
        target_params: Dict[str, float],
        reason: str,
    ) -> None:
        self._pending_set = False
        success = False
        failure_reason = ""
        try:
            result = future.result()
            success = bool(getattr(result, "successful", False))
            failure_reason = str(getattr(result, "reason", ""))
        except Exception as exc:
            failure_reason = str(exc)

        if success:
            state_changed = self._current_state != target_state
            self._current_state = target_state
            self._last_applied_params = dict(target_params)
            if state_changed:
                self._publish_state_event(target_state, reason)
            self._node.get_logger().info(
                "Dynamic nav profile applied "
                f"(state={target_state}, reason={reason}, params={target_params})."
            )
        else:
            self._deferred_state = target_state
            self._deferred_reason = reason
            self._node.get_logger().warn(
                "Dynamic nav profile apply rejected "
                f"(state={target_state}, reason={reason}, detail={failure_reason})."
            )

        if self._deferred_state is None:
            return
        deferred_state = self._deferred_state
        deferred_reason = self._deferred_reason or "dynamic_nav_profile_deferred"
        self._deferred_state = None
        self._deferred_reason = ""
        if success and deferred_state == target_state:
            return
        self._request_state(deferred_state, deferred_reason)

    def _publish_state_event(self, target_state: str, reason: str) -> None:
        payload: Dict[str, Any] = {
            "reason": reason,
            "nav_profile_state": target_state,
        }
        if self._last_sample is not None:
            if self._last_sample.estimated_width_m is not None:
                payload["nav_profile_width_m"] = float(self._last_sample.estimated_width_m)
            if self._last_sample.forward_clear_m is not None:
                payload["nav_profile_forward_clear_m"] = float(self._last_sample.forward_clear_m)
        event_name = (
            "NAV_PROFILE_WIDE_APPLIED"
            if target_state == self.PROFILE_WIDE
            else "NAV_PROFILE_BASELINE_RESTORED"
        )
        self._publish_event_cb(event_name, payload)

    def _on_retry_timer(self) -> None:
        if not self._settings.enabled or self._param_client is None:
            return
        if self._pending_get or self._pending_set:
            return
        if self._baseline_params is None:
            self._ensure_baseline("retry_timer")
            return
        if self._deferred_state is None:
            return
        deferred_state = self._deferred_state
        deferred_reason = self._deferred_reason or "retry_timer"
        self._deferred_state = None
        self._deferred_reason = ""
        self._request_state(deferred_state, deferred_reason, force=True)

    def _publish_deleteall_markers(self) -> None:
        if self._marker_pub is None:
            return
        msg = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        self._marker_pub.publish(msg)

    def _publish_markers(self, scan_msg: LaserScan, sample: ScanSpaceSample) -> None:
        if self._marker_pub is None:
            return

        frame_id = str(scan_msg.header.frame_id or "base_footprint")
        stamp = self._node.get_clock().now().to_msg()
        markers = MarkerArray()

        markers.markers.append(
            self._build_line_marker(
                marker_id=1,
                frame_id=frame_id,
                stamp=stamp,
                ns="nav_profile",
                color=(0.2, 0.9, 0.2, 0.95),
                angle_deg=90.0,
                distance=sample.left_clear_m,
            )
        )
        markers.markers.append(
            self._build_line_marker(
                marker_id=2,
                frame_id=frame_id,
                stamp=stamp,
                ns="nav_profile",
                color=(0.2, 0.6, 1.0, 0.95),
                angle_deg=-90.0,
                distance=sample.right_clear_m,
            )
        )
        markers.markers.append(
            self._build_line_marker(
                marker_id=3,
                frame_id=frame_id,
                stamp=stamp,
                ns="nav_profile",
                color=(1.0, 0.9, 0.2, 0.95),
                angle_deg=0.0,
                distance=sample.forward_clear_m,
            )
        )
        markers.markers.append(
            self._build_text_marker(
                marker_id=10,
                frame_id=frame_id,
                stamp=stamp,
                ns="nav_profile",
                text=self._marker_text(sample),
                z=0.45,
                color=(1.0, 1.0, 1.0, 0.95),
            )
        )
        self._marker_pub.publish(markers)

    def _marker_text(self, sample: ScanSpaceSample) -> str:
        lookahead = None
        if self._last_applied_params is not None:
            lookahead = self._last_applied_params.get(self._PARAM_LOOKAHEAD)
        width_text = (
            f"{sample.estimated_width_m:.2f}" if sample.estimated_width_m is not None else "n/a"
        )
        forward_text = (
            f"{sample.forward_clear_m:.2f}" if sample.forward_clear_m is not None else "n/a"
        )
        lookahead_text = f"{lookahead:.2f}" if isinstance(lookahead, float) else "n/a"
        return (
            f"profile={self._current_state}\n"
            f"width={width_text}m forward={forward_text}m\n"
            f"lookahead={lookahead_text}m"
        )

    @staticmethod
    def _build_line_marker(
        *,
        marker_id: int,
        frame_id: str,
        stamp: Any,
        ns: str,
        color: tuple[float, float, float, float],
        angle_deg: float,
        distance: Optional[float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points.append(Point(x=0.0, y=0.0, z=0.02))
        if distance is None:
            marker.points.append(Point(x=0.0, y=0.0, z=0.02))
            return marker
        radians = math.radians(angle_deg)
        marker.points.append(
            Point(
                x=float(distance) * math.cos(radians),
                y=float(distance) * math.sin(radians),
                z=0.02,
            )
        )
        return marker

    @staticmethod
    def _build_text_marker(
        *,
        marker_id: int,
        frame_id: str,
        stamp: Any,
        ns: str,
        text: str,
        z: float,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = z
        marker.scale.z = 0.12
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        return marker


def _parameter_value_to_python(parameter_value: Any) -> Any:
    param_type = int(getattr(parameter_value, "type", 0))
    if param_type == 1:
        return bool(getattr(parameter_value, "bool_value", False))
    if param_type == 2:
        return int(getattr(parameter_value, "integer_value", 0))
    if param_type == 3:
        return float(getattr(parameter_value, "double_value", 0.0))
    if param_type == 4:
        return str(getattr(parameter_value, "string_value", ""))
    return None
