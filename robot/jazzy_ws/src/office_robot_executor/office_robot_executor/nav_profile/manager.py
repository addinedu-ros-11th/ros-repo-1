import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import LaserScan

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
        if self._settings.enabled:
            self._param_client = AsyncParameterClient(node, controller_node_full_name)
            self._scan_sub = node.create_subscription(
                LaserScan,
                self._settings.scan_topic,
                self._on_scan,
                10,
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
            return
        self._ensure_baseline(reason)

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
        if not self._param_client.wait_for_services(timeout_sec=0.1):
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

    def _request_state(self, target_state: str, reason: str) -> None:
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

        if target_state == self._current_state and self._last_applied_params is not None:
            return

        target_params = self._target_params_for_state(target_state)
        if self._last_applied_params == target_params:
            self._current_state = target_state
            return

        if not self._param_client.wait_for_services(timeout_sec=0.1):
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
