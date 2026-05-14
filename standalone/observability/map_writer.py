"""Shared-memory occupancy-map writer for the live viewer subprocess.

Parent process (mjpython) writes occupancy grid + robot pose + path + goal
to a numpy.memmap. A separate Python subprocess running standalone/viz/map_viewer.py
reads the memmap and renders a matplotlib FuncAnimation window in its OWN
main thread — sidestepping the macOS "GUI must be on main thread" crash
that hits Tk/Qt under mjpython's background pthread.

Single-writer (parent), single-reader (child). frame_id increments each
write so the child can poll cheaply.

File layout (fixed offsets, little-endian):

    Header (256 B):
      0   uint32   magic = 0xCAFE5A11
      4   int32    width
      8   int32    height
      12  float32  resolution (m/cell)
      16  float32  origin_x
      20  float32  origin_y
      24  float32  robot_x
      28  float32  robot_y
      32  float32  robot_yaw
      36  float32  goal_x
      40  float32  goal_y
      44  uint8    has_goal
      45  uint8    closed              (writer signals shutdown)
      46  uint16   _pad
      48  int32    path_len
      52  char[32] status              (e.g. "searching")
      84  uint64   frame_id
      92  ..255    reserved
    Grid:  256 .. 256 + H*W            (int8 — -1/0/100)
    Path:  next .. + MAX_PATH*2*4      (float32 (x, y) pairs)
"""
from __future__ import annotations

import os
import struct
import tempfile
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np

from ..mapping.occupancy_grid import OccupancyMapper

MAGIC = 0xCAFE5A11
HEADER_SIZE = 256
STATUS_OFFSET = 52
STATUS_SIZE = 32
FRAME_OFFSET = 84
DEFAULT_MAX_PATH = 256


def _file_size(width: int, height: int, max_path: int) -> int:
    return HEADER_SIZE + width * height + max_path * 2 * 4


class MapWriter:
    """Writes the live exploration state into a memmap for the viewer subprocess.

    Construction:
        writer = MapWriter(mapper, max_path=256)
        # subprocess: viewer reads from writer.path

    Per-tick API:
        writer.tick(robot_pose)         # rewrite grid + robot pose
        writer.set_path(path_xy_list)   # update overlay path
        writer.set_goal(gx, gy)         # set goal X marker
        writer.clear_goal()
        writer.set_status("executing")  # one of the CFPA2 status strings

    Cleanup:
        writer.close()                  # marks header.closed = 1
    """

    def __init__(
        self,
        mapper: OccupancyMapper,
        *,
        path: Optional[str | Path] = None,
        max_path: int = DEFAULT_MAX_PATH,
    ) -> None:
        self._mapper = mapper
        self._max_path = int(max_path)
        self._width = int(mapper.width)
        self._height = int(mapper.height)
        self._resolution = float(mapper.resolution)
        self._origin_x = float(mapper.origin_x)
        self._origin_y = float(mapper.origin_y)

        if path is None:
            fd, path_str = tempfile.mkstemp(prefix="standalone_map_", suffix=".bin")
            os.close(fd)
            self.path = Path(path_str)
        else:
            self.path = Path(path)

        size = _file_size(self._width, self._height, self._max_path)
        with open(self.path, "wb") as f:
            f.seek(size - 1)
            f.write(b"\0")

        self._mm = np.memmap(self.path, dtype=np.uint8, mode="r+", shape=(size,))

        # Cached buffers (avoid reshape on every tick).
        self._grid_view = self._mm[HEADER_SIZE: HEADER_SIZE + self._width * self._height].view(np.int8)
        self._grid_view = self._grid_view.reshape(self._height, self._width)
        path_off = HEADER_SIZE + self._width * self._height
        path_bytes = self._max_path * 2 * 4
        self._path_view = self._mm[path_off: path_off + path_bytes].view(np.float32)
        self._path_view = self._path_view.reshape(self._max_path, 2)

        self._frame_id = 0
        self._status = "init"
        self._goal: Optional[Tuple[float, float]] = None

        self._write_static_header()
        self.tick((self._origin_x + 0.5 * self._width * self._resolution,
                   self._origin_y + 0.5 * self._height * self._resolution,
                   0.0))

    # ── Header helpers ──────────────────────────────────────────────────────

    def _write_static_header(self) -> None:
        struct.pack_into("<I", self._mm, 0, MAGIC)
        struct.pack_into("<i", self._mm, 4, self._width)
        struct.pack_into("<i", self._mm, 8, self._height)
        struct.pack_into("<f", self._mm, 12, self._resolution)
        struct.pack_into("<f", self._mm, 16, self._origin_x)
        struct.pack_into("<f", self._mm, 20, self._origin_y)

    def _bump_frame(self) -> None:
        self._frame_id += 1
        struct.pack_into("<Q", self._mm, FRAME_OFFSET, self._frame_id)

    # ── Public API ──────────────────────────────────────────────────────────

    def tick(self, robot_pose: Tuple[float, float, float]) -> None:
        """Rewrite grid + robot pose. Call on every map update (~5 Hz)."""
        rx, ry, ryaw = robot_pose
        # Re-sync origin in case the mapper recentred.
        if (float(self._mapper.origin_x) != self._origin_x or
                float(self._mapper.origin_y) != self._origin_y):
            self._origin_x = float(self._mapper.origin_x)
            self._origin_y = float(self._mapper.origin_y)
            struct.pack_into("<f", self._mm, 16, self._origin_x)
            struct.pack_into("<f", self._mm, 20, self._origin_y)

        struct.pack_into("<f", self._mm, 24, float(rx))
        struct.pack_into("<f", self._mm, 28, float(ry))
        struct.pack_into("<f", self._mm, 32, float(ryaw))

        # Build int8 grid same way OccupancyMapper.to_occupancy_grid does,
        # but write directly into the memmap (avoids tolist() round-trip).
        log_odds = self._mapper._log_odds
        observed = self._mapper._observed
        self._grid_view.fill(-1)
        free_mask = observed & (log_odds <= self._mapper._free_thr)
        occ_mask = observed & (log_odds >= self._mapper._occ_thr)
        self._grid_view[free_mask] = 0
        self._grid_view[occ_mask] = 100

        self._bump_frame()

    def set_path(self, waypoints: Iterable[Tuple[float, float]]) -> None:
        wps = list(waypoints)[: self._max_path]
        n = len(wps)
        if n > 0:
            arr = np.asarray(wps, dtype=np.float32)
            self._path_view[:n] = arr
        struct.pack_into("<i", self._mm, 48, n)
        self._bump_frame()

    def clear_path(self) -> None:
        self.set_path([])

    def set_goal(self, gx: float, gy: float) -> None:
        self._goal = (float(gx), float(gy))
        struct.pack_into("<f", self._mm, 36, float(gx))
        struct.pack_into("<f", self._mm, 40, float(gy))
        struct.pack_into("<B", self._mm, 44, 1)
        self._bump_frame()

    def clear_goal(self) -> None:
        self._goal = None
        struct.pack_into("<B", self._mm, 44, 0)
        self._bump_frame()

    def set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        data = status.encode("ascii", errors="replace")[: STATUS_SIZE - 1]
        buf = data + b"\0" * (STATUS_SIZE - len(data))
        self._mm[STATUS_OFFSET: STATUS_OFFSET + STATUS_SIZE] = np.frombuffer(buf, dtype=np.uint8)
        self._bump_frame()

    def close(self) -> None:
        struct.pack_into("<B", self._mm, 45, 1)
        self._bump_frame()
        try:
            self._mm.flush()
        except Exception:
            pass
        try:
            del self._mm
        except Exception:
            pass
