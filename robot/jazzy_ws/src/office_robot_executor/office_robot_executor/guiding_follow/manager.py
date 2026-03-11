from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

from sensor_msgs.msg import LaserScan

from .scan_tracker import GuideFollowerSample, GuideFollowerScanSettings, GuideFollowerScanTracker


@dataclass(frozen=True)
class GuideFollowerSettings:
    enabled: bool
    scan_topic: str
    rear_center_deg: float
    rear_half_width_deg: float
    acquire_min_distance_m: float
    acquire_max_distance_m: float
    keep_min_distance_m: float
    keep_max_distance_m: float
    min_points: int
    min_cluster_width_m: float
    max_cluster_width_m: float
    acquire_confirm_sec: float
    lost_confirm_sec: float


class GuideFollowerManager:
    def __init__(
        self,
        *,
        node,
        settings: GuideFollowerSettings,
        on_detected_cb: Callable[[GuideFollowerSample, bool], None],
        on_lost_cb: Callable[[Optional[GuideFollowerSample]], None],
    ) -> None:
        self._node = node
        self._settings = settings
        self._on_detected_cb = on_detected_cb
        self._on_lost_cb = on_lost_cb
        self._tracker = GuideFollowerScanTracker(
            GuideFollowerScanSettings(
                rear_center_deg=settings.rear_center_deg,
                rear_half_width_deg=settings.rear_half_width_deg,
                acquire_min_distance_m=settings.acquire_min_distance_m,
                acquire_max_distance_m=settings.acquire_max_distance_m,
                keep_min_distance_m=settings.keep_min_distance_m,
                keep_max_distance_m=settings.keep_max_distance_m,
                min_points=settings.min_points,
                min_cluster_width_m=settings.min_cluster_width_m,
                max_cluster_width_m=settings.max_cluster_width_m,
            )
        )
        self._guiding_active = False
        self._motion_active = False
        self._state = "INACTIVE"
        self._last_sample: Optional[GuideFollowerSample] = None
        self._candidate_since_mono: Optional[float] = None
        self._lost_since_mono: Optional[float] = None
        self._scan_sub = self._node.create_subscription(
            LaserScan, settings.scan_topic, self._on_scan, 10
        )

    def set_guiding_active(self, active: bool, reason: str) -> None:
        active = bool(active and self._settings.enabled)
        if active == self._guiding_active:
            if active and self._state == "INACTIVE":
                self._state = "WAITING"
            return
        self._guiding_active = active
        self._candidate_since_mono = None
        self._lost_since_mono = None
        self._motion_active = False if not active else self._motion_active
        if active:
            self._state = "WAITING"
            self._node.get_logger().info(
                f"Guide follower monitor activated (reason={reason}, scan_topic={self._settings.scan_topic})."
            )
        else:
            self._state = "INACTIVE"
            self._last_sample = None
            self._node.get_logger().info(
                f"Guide follower monitor deactivated (reason={reason})."
            )

    def set_motion_active(self, active: bool, reason: str) -> None:
        active = bool(active and self._guiding_active)
        if active == self._motion_active:
            return
        self._motion_active = active
        if not self._guiding_active:
            self._state = "INACTIVE"
            return
        if active:
            self._state = "FOLLOWING"
            self._lost_since_mono = None
        elif self._state == "FOLLOWING":
            self._state = "WAITING"
        self._node.get_logger().info(
            f"Guide follower motion state updated (active={active}, reason={reason}, state={self._state})."
        )

    def get_status_fields(self) -> Dict[str, object]:
        if self._state == "INACTIVE" and self._last_sample is None:
            return {}
        data: Dict[str, object] = {
            "guide_follow_state": self._state,
        }
        if self._last_sample is not None:
            data["guide_follow_distance_m"] = float(self._last_sample.distance_m)
            data["guide_follow_cluster_width_m"] = float(self._last_sample.cluster_width_m)
            data["guide_follow_cluster_points"] = int(self._last_sample.cluster_points)
            data["guide_follow_cluster_center_deg"] = float(self._last_sample.center_angle_deg)
        return data

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._guiding_active:
            return

        now_mono = time.monotonic()
        sample = self._tracker.inspect(msg, tracking_active=self._motion_active or self._state == "LOST")
        if sample is not None:
            self._last_sample = sample
            self._lost_since_mono = None
            if self._state == "FOLLOWING":
                self._candidate_since_mono = now_mono
                return

            if self._candidate_since_mono is None:
                self._candidate_since_mono = now_mono
                return
            if (now_mono - self._candidate_since_mono) < self._settings.acquire_confirm_sec:
                return

            resumed = self._state == "LOST"
            self._state = "FOLLOWING"
            self._candidate_since_mono = now_mono
            self._on_detected_cb(sample, resumed)
            return

        self._candidate_since_mono = None
        if self._state != "FOLLOWING":
            return

        if self._lost_since_mono is None:
            self._lost_since_mono = now_mono
            return
        if (now_mono - self._lost_since_mono) < self._settings.lost_confirm_sec:
            return

        self._state = "LOST"
        self._motion_active = False
        self._lost_since_mono = None
        self._on_lost_cb(self._last_sample)
