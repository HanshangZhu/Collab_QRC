from __future__ import annotations

import math
from typing import Optional


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points


def world_delta_to_local(wx: float, wy: float, robot_yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    lx = cos_yaw * wx + sin_yaw * wy
    ly = -sin_yaw * wx + cos_yaw * wy
    return (lx, ly)


def local_to_world(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    lx: float,
    ly: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    wx = robot_x + cos_yaw * lx - sin_yaw * ly
    wy = robot_y + sin_yaw * lx + cos_yaw * ly
    return (wx, wy)


def local_to_grid(
    lx: float,
    ly: float,
    planner_resolution: float,
    planner_cells: int,
) -> Optional[tuple[int, int]]:
    c = planner_cells // 2
    gx = int(round(lx / planner_resolution)) + c
    gy = int(round(ly / planner_resolution)) + c
    if gx < 0 or gy < 0 or gx >= planner_cells or gy >= planner_cells:
        return None
    return (gx, gy)


def grid_to_local(
    gx: int,
    gy: int,
    planner_resolution: float,
    planner_cells: int,
) -> tuple[float, float]:
    c = planner_cells // 2
    lx = (gx - c) * planner_resolution
    ly = (gy - c) * planner_resolution
    return (lx, ly)


def smooth_path_chaikin(
    path: list[tuple[float, float]],
    iterations: int = 3,
) -> list[tuple[float, float]]:
    """Chaikin corner-cutting: each pass replaces sharp corners with two 25/75% points."""
    for _ in range(iterations):
        if len(path) < 3:
            return path
        smoothed: list[tuple[float, float]] = [path[0]]
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            smoothed.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
            smoothed.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
        smoothed.append(path[-1])
        path = smoothed
    return path


def resample_path(
    path: list[tuple[float, float]],
    spacing: float,
) -> list[tuple[float, float]]:
    """Walk along *path* placing points every *spacing* metres."""
    if len(path) < 2 or spacing <= 1e-6:
        return list(path)
    result: list[tuple[float, float]] = [path[0]]
    carry = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        ux, uy = dx / seg, dy / seg
        pos = 0.0
        remaining = seg
        # Resume from leftover distance of previous segment.
        if carry > 0.0:
            if carry <= remaining:
                pos = carry
                result.append((path[i - 1][0] + pos * ux, path[i - 1][1] + pos * uy))
                remaining -= carry
                carry = 0.0
            else:
                carry -= remaining
                continue
        while remaining >= spacing:
            pos += spacing
            result.append((path[i - 1][0] + pos * ux, path[i - 1][1] + pos * uy))
            remaining -= spacing
        carry = remaining
    # Always include the final point.
    last = path[-1]
    if math.hypot(last[0] - result[-1][0], last[1] - result[-1][1]) > 1e-6:
        result.append(last)
    return result
