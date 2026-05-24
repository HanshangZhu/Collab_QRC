"""ROS-free SLAM-divergence decision for /odom/nav samples.

A pose is "diverged" if any coordinate is non-finite (NaN/inf) or its
magnitude exceeds ``bound`` metres. Used by odom_divergence_monitor.py to
flag trials where Fast-LIO diverged so the benchmark driver can discard +
re-run them.
"""
from __future__ import annotations

import math


def is_diverged(x: float, y: float, bound: float = 60.0) -> bool:
    for v in (x, y):
        if v is None or not math.isfinite(v):
            return True
        if abs(v) > bound:
            return True
    return False
