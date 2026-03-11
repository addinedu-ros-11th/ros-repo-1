from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class GuideFollowerSample:
    distance_m: float
    cluster_width_m: float
    cluster_points: int
    center_angle_deg: float


@dataclass(frozen=True)
class GuideFollowerScanSettings:
    rear_center_deg: float
    rear_half_width_deg: float
    acquire_min_distance_m: float
    acquire_max_distance_m: float
    keep_min_distance_m: float
    keep_max_distance_m: float
    min_points: int
    min_cluster_width_m: float
    max_cluster_width_m: float


class GuideFollowerScanTracker:
    def __init__(self, settings: GuideFollowerScanSettings) -> None:
        self._settings = settings

    def inspect(self, scan: LaserScan, tracking_active: bool) -> Optional[GuideFollowerSample]:
        points = self._extract_sector_points(scan, tracking_active=tracking_active)
        if len(points) < self._settings.min_points:
            return None

        clusters = self._cluster_points(points, scan)
        if not clusters:
            return None

        best = self._select_best_cluster(clusters)
        if best is None:
            return None

        cluster_width_m = self._cluster_width(best)
        if cluster_width_m < self._settings.min_cluster_width_m:
            return None
        if cluster_width_m > self._settings.max_cluster_width_m:
            return None

        distances = sorted(distance for _, distance in best)
        median_distance = distances[len(distances) // 2]
        center_angle_deg = sum(angle for angle, _ in best) / float(len(best))
        return GuideFollowerSample(
            distance_m=float(median_distance),
            cluster_width_m=float(cluster_width_m),
            cluster_points=len(best),
            center_angle_deg=float(center_angle_deg),
        )

    def _extract_sector_points(
        self, scan: LaserScan, *, tracking_active: bool
    ) -> List[Tuple[float, float]]:
        min_distance = (
            self._settings.keep_min_distance_m
            if tracking_active
            else self._settings.acquire_min_distance_m
        )
        max_distance = (
            self._settings.keep_max_distance_m
            if tracking_active
            else self._settings.acquire_max_distance_m
        )

        points: List[Tuple[float, float]] = []
        angle = float(scan.angle_min)
        angle_increment = float(scan.angle_increment)
        for distance in scan.ranges:
            if not math.isfinite(distance):
                angle += angle_increment
                continue
            if distance < min_distance or distance > max_distance:
                angle += angle_increment
                continue
            angle_deg = math.degrees(angle)
            if self._rear_distance_deg(angle_deg) <= self._settings.rear_half_width_deg:
                points.append((angle_deg, float(distance)))
            angle += angle_increment
        return points

    def _cluster_points(
        self, points: Sequence[Tuple[float, float]], scan: LaserScan
    ) -> List[List[Tuple[float, float]]]:
        if not points:
            return []

        sorted_points = sorted(points, key=lambda item: item[0])
        max_gap_deg = max(2.5, abs(math.degrees(float(scan.angle_increment))) * 3.0)
        clusters: List[List[Tuple[float, float]]] = [[sorted_points[0]]]
        previous_angle = sorted_points[0][0]
        for angle_deg, distance in sorted_points[1:]:
            if abs(angle_deg - previous_angle) <= max_gap_deg:
                clusters[-1].append((angle_deg, distance))
            else:
                clusters.append([(angle_deg, distance)])
            previous_angle = angle_deg
        return [cluster for cluster in clusters if len(cluster) >= self._settings.min_points]

    def _select_best_cluster(
        self, clusters: Sequence[Sequence[Tuple[float, float]]]
    ) -> Optional[List[Tuple[float, float]]]:
        best_cluster: Optional[List[Tuple[float, float]]] = None
        best_key: Optional[Tuple[int, float, float]] = None
        for cluster in clusters:
            mean_distance = sum(distance for _, distance in cluster) / float(len(cluster))
            mean_angle = sum(angle for angle, _ in cluster) / float(len(cluster))
            key = (
                len(cluster),
                -self._rear_distance_deg(mean_angle),
                -mean_distance,
            )
            if best_key is None or key > best_key:
                best_key = key
                best_cluster = list(cluster)
        return best_cluster

    @staticmethod
    def _cluster_width(cluster: Sequence[Tuple[float, float]]) -> float:
        if len(cluster) < 2:
            return 0.0
        left_angle_deg, left_distance = cluster[0]
        right_angle_deg, right_distance = cluster[-1]
        angle_delta_rad = math.radians(abs(right_angle_deg - left_angle_deg))
        return math.sqrt(
            max(
                0.0,
                left_distance * left_distance
                + right_distance * right_distance
                - 2.0 * left_distance * right_distance * math.cos(angle_delta_rad),
            )
        )

    def _rear_distance_deg(self, angle_deg: float) -> float:
        normalized = ((angle_deg + 180.0) % 360.0) - 180.0
        return abs(abs(normalized) - 180.0)
