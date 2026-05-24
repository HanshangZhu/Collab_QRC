"""Architecture-agnostic exploration-coverage helpers.

The union of per-robot known-cell sets gives a global coverage measure that
does not depend on any coordinator-only ``/merged_map`` topic, so the same
metric applies to both the centralised and decentralised CFPA2 stacks.

Lives in ``scripts/`` (not an importable module) because go2w_observability is
an ament_cmake scripts-only package; consumers add this directory to sys.path.
"""
from __future__ import annotations

from typing import Iterable


def union_known_cell_count(cell_sets: Iterable[set]) -> int:
    """Number of distinct (ix, iy) cells known by at least one robot."""
    sets = [s for s in cell_sets if s]
    if not sets:
        return 0
    return len(set().union(*sets))
