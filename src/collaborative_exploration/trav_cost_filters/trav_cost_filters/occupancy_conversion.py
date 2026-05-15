"""Pure helpers for converting traversability maps into Nav2 occupancy grids."""

from __future__ import annotations

import math

import numpy as np


def traversability_to_occupancy(
    traversability: np.ndarray,
    *,
    free_threshold: float,
    lethal_threshold: float,
) -> np.ndarray:
    """Convert traversability [0, 1] into OccupancyGrid costs.

    Unknown/non-finite traversability stays -1. Cells above the free threshold
    become 0, cells below the lethal threshold become 100, and the middle band
    is linearly scaled into 1..99.
    """

    if free_threshold <= lethal_threshold:
        raise ValueError("free_threshold must be greater than lethal_threshold")

    trav = np.asarray(traversability, dtype=np.float32)
    cost = np.full(trav.shape, -1, dtype=np.int8)
    valid = np.isfinite(trav)
    if not np.any(valid):
        return cost

    t = np.clip(trav[valid], 0.0, 1.0)
    converted = np.empty(t.shape, dtype=np.int16)

    free = t >= free_threshold
    lethal = t < lethal_threshold
    middle = ~(free | lethal)

    converted[free] = 0
    converted[lethal] = 100
    if np.any(middle):
        mid_cost = np.rint(
            (free_threshold - t[middle])
            / (free_threshold - lethal_threshold)
            * 100.0
        )
        converted[middle] = np.clip(mid_cost, 1, 99).astype(np.int16)

    cost[valid] = converted.astype(np.int8)
    return cost


def stamp_free_disk(
    occupancy: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    center_x: float,
    center_y: float,
    radius_m: float,
    free_value: int = 0,
    max_clear_cost: int = 50,
) -> int:
    """Mark a bounded robot-footprint disk free without clearing obstacles.

    The seed solves the elevation-map blind spot directly below the robot. It
    only clears unknown cells and already-traversable/intermediate cells; costs
    above ``max_clear_cost`` are treated as obstacle evidence and left intact.
    Returns the number of cells changed to ``free_value``.
    """

    if radius_m <= 0.0 or resolution <= 0.0:
        return 0

    grid = np.asarray(occupancy)
    if grid.ndim != 2:
        raise ValueError("occupancy must be a 2D array")

    height, width = grid.shape
    min_x = max(0, int(math.floor((center_x - radius_m - origin_x) / resolution)))
    max_x = min(width - 1, int(math.floor((center_x + radius_m - origin_x) / resolution)))
    min_y = max(0, int(math.floor((center_y - radius_m - origin_y) / resolution)))
    max_y = min(height - 1, int(math.floor((center_y + radius_m - origin_y) / resolution)))

    if min_x > max_x or min_y > max_y:
        return 0

    xs = origin_x + (np.arange(min_x, max_x + 1, dtype=np.float32) + 0.5) * resolution
    ys = origin_y + (np.arange(min_y, max_y + 1, dtype=np.float32) + 0.5) * resolution
    dx2 = (xs[None, :] - center_x) ** 2
    dy2 = (ys[:, None] - center_y) ** 2
    disk = (dx2 + dy2) <= (radius_m * radius_m)

    sub = grid[min_y : max_y + 1, min_x : max_x + 1]
    clearable = (sub < 0) | ((sub >= 0) & (sub <= max_clear_cost))
    change = disk & clearable & (sub != free_value)
    n_changed = int(np.count_nonzero(change))
    if n_changed:
        sub[change] = free_value
    return n_changed


def apply_slope_verified_ramp_override(
    occupancy: np.ndarray,
    *,
    slope: np.ndarray,
    step_residual: np.ndarray,
    min_slope_rad: float,
    max_slope_rad: float,
    max_step_residual_m: float,
    free_value: int = 0,
) -> int:
    """Clear cells that satisfy the continuous-ramp equation.

    A ramp may receive a low multiplicative traversability score when the
    roughness window straddles a slope transition.  The ETH wall/ramp
    discriminator is the residual step after subtracting the continuous
    plane rise, so a finite cell is explicitly traversable when:

        min_slope <= slope <= max_slope
        step_residual <= max_step_residual

    Vertical walls still fail on slope and discontinuous box/wall edges fail
    on residual step.
    """

    grid = np.asarray(occupancy)
    slp = np.asarray(slope, dtype=np.float32)
    step = np.asarray(step_residual, dtype=np.float32)
    if grid.shape != slp.shape or grid.shape != step.shape:
        raise ValueError("occupancy, slope, and step_residual must share shape")

    ramp = (
        np.isfinite(slp)
        & np.isfinite(step)
        & (slp >= float(min_slope_rad))
        & (slp <= float(max_slope_rad))
        & (step <= float(max_step_residual_m))
    )
    change = ramp & (grid != int(free_value))
    n_changed = int(np.count_nonzero(change))
    if n_changed:
        grid[change] = int(free_value)
    return n_changed
