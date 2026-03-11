import math
from dataclasses import dataclass
from typing import Any, List, Optional


@dataclass(frozen=True)
class ScanSpaceSample:
    left_clear_m: Optional[float]
    right_clear_m: Optional[float]
    min_side_clear_m: Optional[float]
    forward_clear_m: Optional[float]
    estimated_width_m: Optional[float]
    valid: bool


def analyze_scan_space(
    scan_msg: Any,
    *,
    robot_width_m: float,
    side_arc_center_deg: float = 90.0,
    side_arc_half_width_deg: float = 6.0,
    forward_arc_half_width_deg: float = 15.0,
    percentile: float = 0.1,
) -> ScanSpaceSample:
    ranges = list(getattr(scan_msg, "ranges", []) or [])
    if not ranges:
        return ScanSpaceSample(None, None, None, None, False)

    angle_min = float(getattr(scan_msg, "angle_min", 0.0))
    angle_increment = float(getattr(scan_msg, "angle_increment", 0.0))
    range_min = float(getattr(scan_msg, "range_min", 0.0))
    range_max = float(getattr(scan_msg, "range_max", 0.0))
    if not math.isfinite(angle_increment) or abs(angle_increment) < 1.0e-9:
        return ScanSpaceSample(None, None, None, None, False)

    side_center_rad = math.radians(side_arc_center_deg)
    side_half_rad = math.radians(side_arc_half_width_deg)
    forward_half_rad = math.radians(forward_arc_half_width_deg)

    left_values: List[float] = []
    right_values: List[float] = []
    forward_values: List[float] = []

    for index, raw_range in enumerate(ranges):
        if raw_range is None:
            continue
        distance = float(raw_range)
        if not math.isfinite(distance):
            continue
        if distance <= max(0.0, range_min) or (range_max > 0.0 and distance > range_max):
            continue

        angle = angle_min + (angle_increment * float(index))
        if _angle_within(angle, side_center_rad, side_half_rad):
            left_values.append(distance)
        if _angle_within(angle, -side_center_rad, side_half_rad):
            right_values.append(distance)
        if _angle_within(angle, 0.0, forward_half_rad):
            forward_values.append(distance)

    left_clear = _percentile(left_values, percentile)
    right_clear = _percentile(right_values, percentile)
    forward_clear = _percentile(forward_values, percentile)
    min_side_clear = None
    if left_clear is not None and right_clear is not None:
        min_side_clear = float(min(left_clear, right_clear))

    estimated_width = None
    if min_side_clear is not None:
        # Use the tighter side as the corridor-width proxy so corners and
        # asymmetric openings are not misclassified as wide corridors.
        estimated_width = float((2.0 * min_side_clear) + max(0.0, robot_width_m))

    return ScanSpaceSample(
        left_clear_m=left_clear,
        right_clear_m=right_clear,
        min_side_clear_m=min_side_clear,
        forward_clear_m=forward_clear,
        estimated_width_m=estimated_width,
        valid=estimated_width is not None and forward_clear is not None,
    )


def _angle_within(angle: float, center: float, half_width: float) -> bool:
    delta = _normalize_angle(angle - center)
    return abs(delta) <= half_width


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _percentile(values: List[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    clamped = min(max(float(fraction), 0.0), 1.0)
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    index = int(math.floor((len(ordered) - 1) * clamped))
    index = max(0, min(len(ordered) - 1, index))
    return float(ordered[index])
