"""System + user prompts for the standalone VLM exploration loop.

Response contract (strict JSON, single object):

    {
      "goal":   {"x": <float>, "y": <float>},   // or null to stay
      "reason": "<short why>",
      "artifact_seen": false,                    // optional, default false
      "artifact_pos": {"x": <float>, "y": <float>},  // only if artifact_seen
      "done": false                              // optional terminal flag
    }

Robot frame conventions:
  - World-frame x/y in metres, matching map axes.
  - Map origin + size are passed in scene JSON so the VLM grounds output
    in the same coordinate system as the rendered image.

The system prompt establishes the mission (artifact-finding exploration).
The user prompt carries the per-tick scene JSON. The image (separate
content block) shows the current occupancy + robot pose + frontiers.
"""
from __future__ import annotations

import json
from typing import Any, Dict


SYSTEM_PROMPT_TEMPLATE = """\
You are an autonomous exploration agent for a ground robot. Your primary job is to
maximise coverage of unknown space as fast as possible. Artifact detection is a
secondary bonus — log anything interesting you see on the way, but never sacrifice
exploration progress to hunt for specific objects.

MISSION
{mission}

INPUTS (every cycle)
Image 1 — top-down OCCUPANCY MAP:
  * gray   = unknown (unmapped) — this is what you want to turn into white
  * white  = free (already explored)
  * black  = obstacles / walls
  * red dot + arrow = robot position + heading
  * yellow circles = frontier candidates — each is A*-reachable and pre-validated
  * green X = last active goal
  * blue trail = robot's path so far

Image 2 — ROBOT FRONT CAMERA (RGB, if provided):
  * First-person view from the robot's forward-facing camera.
  * Scan it for brightly colored objects: a RED sphere, a BLUE box, a YELLOW
    cylinder, or a GREEN sphere. They are ~0.18 m across and placed at ~0.5 m
    height, so they should appear as small but vivid patches of solid color
    against the gray/white environment.
  * If any such object is visible — even partially — call log_artifact(x, y,
    description) ONCE. Estimate world (x, y) from the robot pose + bearing.
  * Do NOT call log_artifact for walls, floor, or the robot itself.

DECISION PROCEDURE (follow in order)
1. Look at the map image. Identify which frontier has the LARGEST gray region
   nearby (most unknown space reachable from it). That is your exploration goal.
2. Cross-check with the "info_gain" values in frontier_candidates JSON — higher
   info_gain = more unknown area nearby. Prefer the highest unless the map image
   shows a clearly better option (e.g. a wide open corridor vs a tight corner).
3. Avoid any frontier within 0.5 m of recent_failed_goals.
4. The frontier_candidates list is ALREADY A*-validated — do NOT call validate_path
   on them. Only call validate_path if you want to try a point NOT in the list.
5. If the camera shows a colored object or notable artifact, call log_artifact()
   with your best estimate of its world (x, y). This is the ONLY tool call you
   normally need per cycle.
6. Output the final JSON.

FINAL RESPONSE FORMAT
Output ONLY a single JSON object. NO markdown, NO arrays, NO prose.
Start directly with {{ :

  {{
    "goal":          {{"x": <float>, "y": <float>}} | null,
    "reason":        "<one sentence: which frontier chosen and why>",
    "artifact_seen": <bool, default false>,
    "artifact_pos":  {{"x": <float>, "y": <float>}} | null,
    "done":          <bool, default false>
  }}

Rules:
1. "goal" must be one of the frontier_candidates (x, y) values from the JSON.
2. AVOID any point within 0.5 m of recent_failed_goals.
3. If frontier_candidates is empty, output done: true.
4. If no frontier is reachable (all failed), output goal: null, reason: "hold".
5. Numeric values only — no strings, no Python expressions, no arrays.
"""


def build_system_prompt(mission: str = "Explore the environment and locate any visible artifacts (colored markers, boxes, objects of interest). Report their world coordinates when you spot them.") -> str:
    return SYSTEM_PROMPT_TEMPLATE.format(mission=mission)


def build_user_prompt(scene: Dict[str, Any], max_chars: int = 4000) -> str:
    """Compose the user-message text accompanying the image."""
    blob = json.dumps(scene, indent=2, default=str)
    if len(blob) > max_chars:
        cands = scene.get("frontier_candidates", [])
        if len(cands) > 6:
            scene = {**scene, "frontier_candidates": cands[:6],
                     "frontier_candidates_truncated": len(cands)}
            blob = json.dumps(scene, indent=2, default=str)
    if len(blob) > max_chars:
        blob = blob[:max_chars] + "\n... [truncated]"

    cands = scene.get("frontier_candidates", [])
    best = max(cands, key=lambda c: c.get("info_gain", 0), default=None) if cands else None
    # Best-hint adapts to ablation: when info_gain is stripped (Pri 4),
    # fall back to a neutral hint that doesn't leak the heuristic.
    if best is None:
        best_hint = "No frontier candidates — output done: true."
    elif "info_gain" in best:
        best_hint = (f"Highest info_gain candidate: ({best['x']}, {best['y']}) "
                     f"info_gain={best['info_gain']:.1f} m²")
    else:
        best_hint = f"Frontier candidates available: {len(cands)} — pick one."

    return (
        "Current scene snapshot:\n"
        "```json\n"
        f"{blob}\n"
        "```\n\n"
        f"Hint: {best_hint}\n\n"
        "Steps:\n"
        "1. EXPLORATION: pick the frontier_candidate with the highest info_gain "
        "(most unknown area reachable). Use the map image to visually confirm it "
        "points toward a large gray region, not a dead-end corner.\n"
        "2. ARTIFACT: carefully examine Image 2 (camera). If you see a brightly "
        "colored object — RED sphere, BLUE box, YELLOW cylinder, or GREEN sphere "
        "— call log_artifact(x, y, description) once with your best world-coordinate "
        "estimate. These are small (~18 cm) but vividly colored against the gray scene.\n"
        "3. Output ONLY the final JSON — start directly with {."
    )


# ── Response parsing ─────────────────────────────────────────────────────────

def parse_response(raw: str) -> Dict[str, Any]:
    """Parse VLM response into a normalized dict.

    Returns: {goal: (x,y) or None, reason: str, artifact_seen: bool,
              artifact_pos: (x,y) or None, done: bool}
    """
    from .backend import extract_json_object
    obj = extract_json_object(raw)
    if not isinstance(obj, dict):
        return {"goal": None, "reason": f"parse_error: {raw[:80]!r}",
                "artifact_seen": False, "artifact_pos": None, "done": False}

    goal_obj = obj.get("goal")
    goal_xy = None
    if isinstance(goal_obj, dict):
        try:
            goal_xy = (float(goal_obj["x"]), float(goal_obj["y"]))
        except (KeyError, TypeError, ValueError):
            goal_xy = None

    art_obj = obj.get("artifact_pos")
    art_xy = None
    if isinstance(art_obj, dict):
        try:
            art_xy = (float(art_obj["x"]), float(art_obj["y"]))
        except (KeyError, TypeError, ValueError):
            art_xy = None

    return {
        "goal": goal_xy,
        "reason": str(obj.get("reason", "") or ""),
        "artifact_seen": bool(obj.get("artifact_seen", False)),
        "artifact_pos": art_xy,
        "done": bool(obj.get("done", False)),
    }
