from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import wrap_angle


@dataclass
class ScanMetrics:
    min_front: float
    left_push: float
    right_push: float
    rear_clearance: float


class ScanAnalyzer:
    def analyze(self, scan, cfg) -> ScanMetrics:
        min_front = float("inf")
        left_push_sum = 0.0
        right_push_sum = 0.0
        left_count = 0
        right_count = 0

        angle = scan.angle_min
        for r in scan.ranges:
            if not math.isfinite(r) or r < 0.05:
                angle += scan.angle_increment
                continue
            abs_a = abs(angle)

            if abs_a < cfg.front_half and r < min_front:
                min_front = r

            if abs_a < cfg.side_half and r < cfg.obstacle_slow_dist:
                force = (cfg.obstacle_slow_dist - r) / cfg.obstacle_slow_dist
                if angle > 0:
                    left_push_sum += force
                    left_count += 1
                else:
                    right_push_sum += force
                    right_count += 1
            angle += scan.angle_increment

        left_push = left_push_sum / left_count if left_count > 0 else 0.0
        right_push = right_push_sum / right_count if right_count > 0 else 0.0
        rear_clearance = self.rear_clearance(scan)

        # DEBUG: Print scan analysis
        # print(f"DEBUG: min_front={min_front:.3f} left={left_push:.3f} right={right_push:.3f} rear={rear_clearance:.3f}")

        return ScanMetrics(
            min_front=min_front,
            left_push=left_push,
            right_push=right_push,
            rear_clearance=rear_clearance,
        )

    def rear_clearance(self, scan) -> float:
        center = math.pi
        half = math.radians(35.0)
        best = float("inf")
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and r >= 0.05:
                wrapped = abs(wrap_angle(angle - center))
                if wrapped <= half and r < best:
                    best = r
            angle += scan.angle_increment
        return best if math.isfinite(best) else 0.0
