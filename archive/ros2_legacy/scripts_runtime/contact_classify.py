"""Shared contact-classification + tilt utilities.

Single source of truth for how `/mujoco/contacts` geom-pair strings get
labeled into {robot A, robot B, outer wall, interior obstacle, ground}.
Used by both:
    - scripts/runtime/dual_robot_collision_monitor.py  (live debug stream)
    - scripts/bench/session_reporter.py                (per-trial JSON)
so that "did the robot scuff anything?" gives the same answer in both
streams. Before this module existed each script had its own hard-coded
WALL_PREFIXES allowlist that silently dropped every demo3_mixed interior
obstacle (sw_*, ne_*, cross_*, zigzag_*, nw_pillar_*, box_obstacle_*) —
contacts with those geoms were classified as "world" alongside `ground`
and filtered out, producing a perpetually-zero `wall_contact_count`
even when the robot was wedged against an interior box.

Also provides the canonical body-tilt metric (gimbal-lock-free) and
threshold constants so a flip is detected the same way everywhere.
"""
from __future__ import annotations
import math


# ── Robot self-collision parts ────────────────────────────────────────
# Demo3_mixed naming convention:
#   robot_a (Go2W, wheels)  : geoms are bare (`base_collision`, `FL_*`, ...)
#   robot_b (Go2 menagerie) : geoms are `b_`-prefixed (`b_base_collision`,
#                              `b_FL_thigh_collision`, ...)
# The bare 2-letter pad names ("FL"/"FR"/"RL"/"RR") are Go2 menagerie_foot
# geoms — they appear in the MJCF as e.g. `<geom name="b_RL"
# class="menagerie_foot"/>`. Without these in the prefix list,
# classify("b_RL") falls through to "obstacle" and B-thigh × B-foot
# self-collisions during gait get mis-counted as obstacle scuffs.
ROBOT_PART_PREFIXES = (
    "base_collision",
    "head_upper_collision",
    "head_lower_collision",
    "FL_hip_collision", "FL_thigh_collision",
    "FL_calf_upper_collision", "FL_calf_lower_collision", "FL_wheel_collision",
    "FR_hip_collision", "FR_thigh_collision",
    "FR_calf_upper_collision", "FR_calf_lower_collision", "FR_wheel_collision",
    "RL_hip_collision", "RL_thigh_collision",
    "RL_calf_upper_collision", "RL_calf_lower_collision", "RL_wheel_collision",
    "RR_hip_collision", "RR_thigh_collision",
    "RR_calf_upper_collision", "RR_calf_lower_collision", "RR_wheel_collision",
    # Go2 menagerie_foot pads (bare names — only appear as `b_FL` etc.).
    "FL", "FR", "RL", "RR",
)


# ── Outer scene walls (the 4-sided arena boundary) ────────────────────
# Distinguished from interior obstacles only for reporting clarity;
# both kinds latch the `ever_touched_anything` bit.
WALL_PREFIXES = ("wall_", "divider_")


# ── Geoms that are safe to contact ────────────────────────────────────
# The ground plane (foot-on-floor at every step) and known harmless
# props. Scene-specific obstacle names ("box_obstacle_*", "sw_*", "ne_*"
# etc.) used to live here as an allowlist by mistake — they're real
# obstacles, contact with them IS a scuff.
GROUND_OR_HARMLESS_GEOMS = {
    "ground",
    "green_marker_1", "green_marker_2", "green_marker_3",
}


# ── Tilt / tip-over (canonical) ───────────────────────────────────────
# Body-Z axis vs world-up angle, with hold time. Single source of truth;
# both consumers should reference these constants instead of redefining.
# See `tilt_from_quat_deg` for math + rationale.
#
# Two latching levels to distinguish "literally on its back" from "stuck
# leaned over but not flipped":
#
#   TIP_THRESHOLD_DEG (70°)         — full tip. Robot is on its side or
#     held ≥ TIP_HOLD_SEC (1.0 s)     beyond. Mission failure.
#
#   TILT_DEGRADED_DEG (30°)          — degraded posture. Robot is leaned
#     held ≥ TILT_DEGRADED_HOLD_SEC    far past gait-normal (≤ ~10°) but
#     (5.0 s)                          not flipped. Usually means a body
#                                      part is hung up on something — A
#                                      stationary at 59° propped against
#                                      wall_north is the canonical case.
#                                      Planner can't recover from this
#                                      (and shouldn't try); the value of
#                                      flagging it is observability, so
#                                      an operator / agent reading the
#                                      log knows the robot needs
#                                      external intervention.
TIP_THRESHOLD_DEG = 70.0
TIP_HOLD_SEC = 1.0
TILT_DEGRADED_DEG = 30.0
TILT_DEGRADED_HOLD_SEC = 5.0


def _is_wall_geom(name: str) -> bool:
    return name.startswith(WALL_PREFIXES)


def classify(geom_name: str) -> str:
    """Return one of:
        'A'        — robot A self-collision geom
        'B'        — robot B self-collision geom (b_-prefixed in MJCF)
        'wall'     — outer scene boundary wall (wall_*, divider_*)
        'obstacle' — anything else with a real geom name (interior box,
                     pillar, divider, ramp, prop) — i.e. something the
                     robot should not scuff
        'ground'   — the ground plane / harmless markers / empty name;
                     foot-on-floor contacts go here and get filtered out
    """
    if not geom_name:
        return "ground"
    if geom_name in GROUND_OR_HARMLESS_GEOMS:
        return "ground"
    if geom_name.startswith("b_"):
        rest = geom_name[2:]
        if any(rest == p or rest.startswith(p) for p in ROBOT_PART_PREFIXES):
            return "B"
    if any(geom_name == p or geom_name.startswith(p) for p in ROBOT_PART_PREFIXES):
        return "A"
    if _is_wall_geom(geom_name):
        return "wall"
    return "obstacle"


def roll_pitch_yaw_from_quat(q) -> tuple[float, float, float]:
    """Roll/pitch/yaw from a quaternion-with-attrs object (msg.orientation
    style). Kept for human-readable peak_roll/peak_pitch reporting; do
    NOT use for tip detection (gimbal lock at pitch=±90°)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def tilt_from_quat_deg(q) -> float:
    """Angle between the body-Z axis (in world frame) and world-up.
    The body-Z axis expressed in world coords is the third column of
    R(q): bz_world.z = 1 − 2(qx² + qy²). dot product with world-up
    (0,0,1) is just bz_world.z, so tilt = acos(bz_world.z).
    Single-valued, monotonic, no gimbal lock — 0° upright, 90° on its
    side, 180° upside-down. Use this for tip detection instead of
    decomposing to roll/pitch.
    """
    rz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    rz = max(-1.0, min(1.0, rz))
    return math.degrees(math.acos(rz))
