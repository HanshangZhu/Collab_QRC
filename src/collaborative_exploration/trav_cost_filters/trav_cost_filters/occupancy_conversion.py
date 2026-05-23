"""Pure helpers for converting traversability maps into Nav2 occupancy grids."""

from __future__ import annotations

import math

import numpy as np


def grid_map_layer_to_world_array(
    data: np.ndarray,
    *,
    height: int,
    width: int,
    dtype=np.float32,
) -> np.ndarray:
    """Convert a GridMap flat layer into world XY row/column convention.

    elevation_mapping_cupy publishes GridMap data with both axes ordered from
    max to min.  The planning code uses row=y(min→max), col=x(min→max), matching
    OccupancyGrid indexing and the selector geometry helpers.
    """

    expected = int(height) * int(width)
    arr = np.asarray(data, dtype=dtype)
    if arr.size != expected:
        raise ValueError(f"GridMap layer has {arr.size} values, expected {expected}")
    return arr.reshape(int(height), int(width))[::-1, ::-1]


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
    unknown_only: bool = False,
) -> int:
    """Mark a bounded robot-footprint disk free without clearing obstacles.

    The seed solves the elevation-map blind spot directly below the robot. It
    only clears unknown cells and already-traversable/intermediate cells; costs
    above ``max_clear_cost`` are treated as obstacle evidence and left intact.
    Returns the number of cells changed to ``free_value``.

    When ``unknown_only`` is True, ONLY unknown cells (cost < 0) are cleared and
    every cell that carries a measurement (cost >= 0, wall or floor) is left at
    its observed value, regardless of ``max_clear_cost``. This fills the
    geometric blind cone (unseen ground the robot is standing on) without
    erasing near walls the sensor *can* see (Mid-360 with -15 deg tilt: empty
    return hole ~0.43 m, but flat ground first returns ~1.25 m; walls return
    from ~0.43 m). A plain free-disk out to the ground blind cone would wipe
    those scannable near walls; unknown_only does not.
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


def apply_cliff_proximity_cost(
    occupancy: np.ndarray,
    *,
    step_height: np.ndarray,
    resolution: float,
    proximity_radius_m: float,
    step_threshold_m: float,
    step_saturation_m: float,
    max_cost: int,
) -> int:
    """Raise cost for known cells near a local height discontinuity.

    This models stability margin around cliffs/platform edges separately from
    traversability.  A platform cell can be locally flat and still be unsafe if
    a neighbouring cell has a large vertical drop within the robot's support
    margin.  Unknown cells remain unknown and lethal cells stay lethal.
    """

    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if proximity_radius_m <= 0.0:
        return 0
    if step_saturation_m <= step_threshold_m:
        raise ValueError("step_saturation_m must be greater than step_threshold_m")
    if max_cost < 0 or max_cost > 100:
        raise ValueError("max_cost must be in [0, 100]")

    grid = np.asarray(occupancy)
    step = np.asarray(step_height, dtype=np.float32)
    if grid.ndim != 2 or step.ndim != 2:
        raise ValueError("occupancy and step_height must be 2D arrays")
    if grid.shape != step.shape:
        raise ValueError("occupancy and step_height must share shape")

    finite_step = np.where(np.isfinite(step), step, -np.inf)
    if not np.any(np.isfinite(finite_step)):
        return 0

    before = grid.copy()
    local_max = np.full(step.shape, -np.inf, dtype=np.float32)
    radius_cells = int(math.ceil(proximity_radius_m / resolution))

    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * resolution, dy * resolution) > proximity_radius_m + 1e-9:
                continue

            src_y0 = max(0, -dy)
            src_y1 = min(step.shape[0], step.shape[0] - dy)
            src_x0 = max(0, -dx)
            src_x1 = min(step.shape[1], step.shape[1] - dx)
            if src_y0 >= src_y1 or src_x0 >= src_x1:
                continue

            dst_y0 = src_y0 + dy
            dst_y1 = src_y1 + dy
            dst_x0 = src_x0 + dx
            dst_x1 = src_x1 + dx
            local_max[dst_y0:dst_y1, dst_x0:dst_x1] = np.maximum(
                local_max[dst_y0:dst_y1, dst_x0:dst_x1],
                finite_step[src_y0:src_y1, src_x0:src_x1],
            )

    denom = step_saturation_m - step_threshold_m
    risk = np.clip((local_max - step_threshold_m) / denom, 0.0, 1.0)
    risk_cost = np.rint(risk * int(max_cost)).astype(np.int16)

    known = grid >= 0
    raised = known & (risk_cost > grid.astype(np.int16))
    if np.any(raised):
        grid[raised] = risk_cost[raised].astype(grid.dtype, copy=False)
    return int(np.count_nonzero(grid != before))


def project_rolling_grid_to_fixed_grid(
    rolling_occupancy: np.ndarray,
    fixed_occupancy: np.ndarray,
    occupied_hits: np.ndarray | None,
    *,
    rolling_origin_x: float,
    rolling_origin_y: float,
    fixed_origin_x: float,
    fixed_origin_y: float,
    resolution: float,
    unknown_clears_history: bool = False,
    occupied_cost_threshold: int = 100,
    free_cost_threshold: int = 30,
    occupied_confirm_hits: int = 2,
    occupied_clear_hits: int = 0,
    occupied_hit_increment: int = 1,
    free_hit_decrement: int = 1,
    max_hit_count: int = 10,
) -> int:
    """Project a robot-centered rolling occupancy grid into a fixed world grid.

    elevation_mapping_cupy maintains a rolling local map centered on the robot.
    Nav2 StaticLayer and RViz need a stable world-frame OccupancyGrid, so each
    incoming cell is re-indexed by world coordinates before it updates the fixed
    output grid.

    Unknown rolling cells do not erase fixed-grid history by default. Lethal
    observations use a small hit counter so one-frame LiDAR noise does not turn
    into black speckles, while repeated wall observations still become lethal.
    """

    if resolution <= 0.0:
        raise ValueError("resolution must be positive")

    rolling = np.asarray(rolling_occupancy, dtype=np.int8)
    fixed = np.asarray(fixed_occupancy)
    if rolling.ndim != 2 or fixed.ndim != 2:
        raise ValueError("rolling_occupancy and fixed_occupancy must be 2D arrays")
    if occupied_hits is not None and occupied_hits.shape != fixed.shape:
        raise ValueError("occupied_hits must match fixed_occupancy shape")

    before = fixed.copy()
    src_h, src_w = rolling.shape
    dst_h, dst_w = fixed.shape

    xs = rolling_origin_x + (np.arange(src_w, dtype=np.float64) + 0.5) * resolution
    ys = rolling_origin_y + (np.arange(src_h, dtype=np.float64) + 0.5) * resolution
    dst_x = np.floor((xs - fixed_origin_x) / resolution + 1e-6).astype(np.int64)
    dst_y = np.floor((ys - fixed_origin_y) / resolution + 1e-6).astype(np.int64)

    in_x = (dst_x >= 0) & (dst_x < dst_w)
    in_y = (dst_y >= 0) & (dst_y < dst_h)
    if not np.any(in_x) or not np.any(in_y):
        return 0

    src_rows = np.nonzero(in_y)[0]
    src_cols = np.nonzero(in_x)[0]
    dst_rows = dst_y[src_rows]
    dst_cols = dst_x[src_cols]

    vals = rolling[np.ix_(src_rows, src_cols)]
    yy, xx = np.meshgrid(dst_rows, dst_cols, indexing="ij")

    if unknown_clears_history:
        unknown = vals < 0
        if np.any(unknown):
            fixed[yy[unknown], xx[unknown]] = -1
            if occupied_hits is not None:
                occupied_hits[yy[unknown], xx[unknown]] = 0

    observed = vals >= 0
    if not np.any(observed):
        return int(np.count_nonzero(fixed != before))

    obs_y = yy[observed]
    obs_x = xx[observed]
    obs_vals = vals[observed].astype(np.int8, copy=False)

    lethal = obs_vals >= int(occupied_cost_threshold)
    free = obs_vals <= int(free_cost_threshold)
    middle = ~(lethal | free)

    if occupied_hits is None:
        fixed[obs_y, obs_x] = obs_vals
        return int(np.count_nonzero(fixed != before))

    if np.any(lethal):
        ly = obs_y[lethal]
        lx = obs_x[lethal]
        new_hits = np.minimum(
            int(max_hit_count),
            occupied_hits[ly, lx].astype(np.int16) + int(occupied_hit_increment),
        )
        occupied_hits[ly, lx] = new_hits
        confirmed = new_hits >= int(occupied_confirm_hits)
        if np.any(confirmed):
            fixed[ly[confirmed], lx[confirmed]] = 100

    nonlethal = free | middle
    if np.any(nonlethal):
        ny = obs_y[nonlethal]
        nx = obs_x[nonlethal]
        nvals = obs_vals[nonlethal]
        new_hits = np.maximum(
            0,
            occupied_hits[ny, nx].astype(np.int16) - int(free_hit_decrement),
        )
        occupied_hits[ny, nx] = new_hits

        previous = fixed[ny, nx]
        can_update = (previous != 100) | (new_hits <= int(occupied_clear_hits))
        if np.any(can_update):
            fixed[ny[can_update], nx[can_update]] = nvals[can_update]

    return int(np.count_nonzero(fixed != before))


def apply_rectangular_workspace_mask(
    occupancy: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    wall_thickness_m: float = 0.0,
    outside_value: int = -1,
    wall_value: int = 100,
) -> int:
    """Apply a deterministic rectangular workspace boundary to an occupancy map."""

    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    if max_x <= min_x or max_y <= min_y:
        raise ValueError("workspace max bounds must be greater than min bounds")

    grid = np.asarray(occupancy)
    if grid.ndim != 2:
        raise ValueError("occupancy must be a 2D array")

    before = grid.copy()
    height, width = grid.shape
    xs = origin_x + (np.arange(width, dtype=np.float64) + 0.5) * resolution
    ys = origin_y + (np.arange(height, dtype=np.float64) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)

    inside = (xx >= min_x) & (xx <= max_x) & (yy >= min_y) & (yy <= max_y)
    grid[~inside] = int(outside_value)

    if wall_thickness_m > 0.0:
        wall = inside & (
            (xx <= min_x + wall_thickness_m)
            | (xx >= max_x - wall_thickness_m)
            | (yy <= min_y + wall_thickness_m)
            | (yy >= max_y - wall_thickness_m)
        )
        grid[wall] = int(wall_value)

    return int(np.count_nonzero(grid != before))
