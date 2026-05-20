#!/usr/bin/env python3
"""Map merge helpers for M-TARE coordinator."""

from __future__ import annotations

from typing import Optional

from ..core.ros_compat import OccupancyGrid, Odometry


def world_to_grid(msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
    gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
    gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
    if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
        return None
    return (gx, gy)


def grid_to_world(msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
    return (
        msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
        msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
    )


def grid_index(x: int, y: int, w: int) -> int:
    return y * w + x


def copy_map(src: OccupancyGrid) -> OccupancyGrid:
    out = OccupancyGrid()
    out.header = src.header
    out.info = src.info
    out.data = list(src.data)
    return out


def merge_cell_value(
    dst_value: int,
    src_value: int,
    *,
    unknown_value: int,
    free_value: int,
    occ_threshold: int,
) -> int:
    if src_value == unknown_value:
        return dst_value

    src_occ = src_value >= occ_threshold
    dst_occ = dst_value >= occ_threshold

    # Occupied evidence dominates free/unknown.
    if src_occ:
        return src_value
    if dst_occ:
        return dst_value

    # Prefer known over unknown.
    if dst_value == unknown_value:
        return src_value

    # Preserve explicit free values when available.
    if src_value == free_value:
        return free_value
    if dst_value == free_value:
        return free_value

    return src_value


def overlay_map(
    *,
    dst: OccupancyGrid,
    src: OccupancyGrid,
    unknown_value: int,
    free_value: int,
    occ_threshold: int,
    center_xy: Optional[tuple[float, float]] = None,
    radius_m: float = 0.0,
) -> None:
    sw = int(src.info.width)
    sh = int(src.info.height)
    if sw <= 0 or sh <= 0:
        return

    min_x = 0
    min_y = 0
    max_x = sw - 1
    max_y = sh - 1
    if center_xy is not None and radius_m > 0.0:
        cx, cy = center_xy
        g0 = world_to_grid(src, cx - radius_m, cy - radius_m)
        g1 = world_to_grid(src, cx + radius_m, cy + radius_m)
        if g0 is None and g1 is None:
            return
        if g0 is not None:
            min_x = max(min_x, g0[0])
            min_y = max(min_y, g0[1])
        if g1 is not None:
            max_x = min(max_x, g1[0])
            max_y = min(max_y, g1[1])
        if min_x > max_x or min_y > max_y:
            return

    dw = int(dst.info.width)
    ddata = list(dst.data)
    sdata = src.data

    for gy in range(min_y, max_y + 1):
        srow = gy * sw
        for gx in range(min_x, max_x + 1):
            sidx = srow + gx
            sval = sdata[sidx]
            if sval == unknown_value:
                continue
            wx, wy = grid_to_world(src, gx, gy)
            dg = world_to_grid(dst, wx, wy)
            if dg is None:
                continue
            didx = grid_index(dg[0], dg[1], dw)
            ddata[didx] = merge_cell_value(
                ddata[didx],
                int(sval),
                unknown_value=unknown_value,
                free_value=free_value,
                occ_threshold=occ_threshold,
            )

    dst.data = ddata


def build_fallback_map(
    *,
    namespaces: list[str],
    maps: dict[str, OccupancyGrid],
    unknown_value: int,
    free_value: int,
    occ_threshold: int,
) -> Optional[OccupancyGrid]:
    base: Optional[OccupancyGrid] = None
    for ns in namespaces:
        msg = maps.get(ns)
        if msg is not None:
            base = msg
            break
    if base is None:
        return None

    merged = copy_map(base)
    for ns in namespaces:
        src = maps.get(ns)
        if src is None or src is base:
            continue
        overlay_map(
            dst=merged,
            src=src,
            unknown_value=unknown_value,
            free_value=free_value,
            occ_threshold=occ_threshold,
        )
    return merged


def build_shared_with_local_patches(
    *,
    shared_map: OccupancyGrid,
    namespaces: list[str],
    maps: dict[str, OccupancyGrid],
    odoms: dict[str, Odometry],
    local_patch_radius_m: float,
    unknown_value: int,
    free_value: int,
    occ_threshold: int,
) -> OccupancyGrid:
    if local_patch_radius_m <= 0.0:
        return shared_map

    patched = copy_map(shared_map)
    for ns in namespaces:
        src = maps.get(ns)
        od = odoms.get(ns)
        if src is None or od is None:
            continue
        center = (float(od.pose.pose.position.x), float(od.pose.pose.position.y))
        overlay_map(
            dst=patched,
            src=src,
            center_xy=center,
            radius_m=local_patch_radius_m,
            unknown_value=unknown_value,
            free_value=free_value,
            occ_threshold=occ_threshold,
        )
    return patched
