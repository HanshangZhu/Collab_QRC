"""Tool factory for the LangGraph VLM exploration agent.

Each make_tools() call returns four StructuredTools bound to the live
session state via closures — no global state, no thread-shared mapper writes.

Tools receive a *snapshot* of the grid/pose/frontiers taken at dispatch time
so they are safe to call from the background LangGraph worker thread.
"""
from __future__ import annotations

import json
from typing import Callable, List, Optional, Tuple

from langchain_core.tools import StructuredTool
from pydantic import BaseModel, Field


# ── Pydantic input schemas ────────────────────────────────────────────────────

class _ValidatePathInput(BaseModel):
    x: float = Field(..., description="World x-coordinate in metres")
    y: float = Field(..., description="World y-coordinate in metres")


class _LogArtifactInput(BaseModel):
    x: float = Field(..., description="World x-coordinate of the artifact")
    y: float = Field(..., description="World y-coordinate of the artifact")
    description: str = Field(
        ..., description="Short description of what was observed"
    )


# ── Factory ──────────────────────────────────────────────────────────────────

def make_tools(
    *,
    ros_grid_snapshot,                              # OccupancyGrid snapshot from dispatch time
    planner,                                        # AStarPlanner instance
    pose_snapshot: Tuple[float, float, float],      # (x, y, yaw) at dispatch time
    frontiers_snapshot: List[dict],                 # pre-computed frontier list (kept for compat)
    mapper,                                         # OccupancyMapper (kept for compat)
    on_artifact: Callable[[float, float, str], None],
) -> list:
    """Return two exploration tools bound to this dispatch cycle's state.

    Intentionally lean: frontier_candidates and coverage stats are already
    embedded in the scene JSON the model receives as text, so list_frontiers()
    and get_coverage() would be redundant extra API round-trips.  Only
    validate_path (real A* check) and log_artifact (side-effect) are kept.
    This halves the average API calls per LangGraph cycle, which matters on
    rate-limited free tiers (Groq 30 RPM, Gemini 15 RPM).
    """

    _grid = ros_grid_snapshot
    _pose_xy = (pose_snapshot[0], pose_snapshot[1])

    def _validate_path(x: float, y: float) -> str:
        """Check whether (x, y) is reachable via A*. Returns {reachable, path_steps}."""
        path = planner.plan(_grid, _pose_xy, (x, y))
        if path:
            return json.dumps({"reachable": True, "path_steps": len(path)})
        return json.dumps({"reachable": False, "path_steps": 0,
                           "hint": "Try a nearby frontier candidate instead."})

    def _log_artifact(x: float, y: float, description: str) -> str:
        """Record a spotted artifact at world (x, y). Returns confirmation."""
        on_artifact(float(x), float(y), description)
        return json.dumps({
            "logged": True, "x": round(x, 3), "y": round(y, 3),
            "description": description,
        })

    return [
        StructuredTool.from_function(
            func=_validate_path,
            name="validate_path",
            description=(
                "Check if world position (x, y) is reachable by the A* path planner "
                "from the robot's current position. "
                "Returns {reachable: bool, path_steps: int}. "
                "Call this before committing to any goal to confirm it is navigable."
            ),
            args_schema=_ValidatePathInput,
        ),
        StructuredTool.from_function(
            func=_log_artifact,
            name="log_artifact",
            description=(
                "Log a spotted artifact at world position (x, y) with a short description. "
                "Call this whenever the camera image shows an object of interest — "
                "colored marker, box, person, or any mission-relevant object. "
                "Returns {logged: true}."
            ),
            args_schema=_LogArtifactInput,
        ),
    ]
