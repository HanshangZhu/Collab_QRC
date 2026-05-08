#!/usr/bin/env bash
# Launch nav test on pure Go2 (no wheels, spherical feet) + Livox MID-360 +
# Fast-LIO2 + FAR planner. Scene is demo1_go2 (12×8m).
#
# Usage:
#   ./scripts/nav_test_go2.sh                       # headless (FAR default)
#   ./scripts/nav_test_go2.sh gui:=true             # with MuJoCo GUI
#   ./scripts/nav_test_go2.sh gui:=true rviz:=true  # + RViz
#
# Nav2 MPPI stack (added 2026-05-02 PM):
#   ./scripts/nav_test_go2.sh gui:=true rviz:=true nav_backend:=nav2_mppi
#   ./scripts/nav_test_go2.sh gui:=true rviz:=true nav_backend:=nav2_mppi \
#       holonomic_profile:=se2_holonomic
#       # SmacPlannerLattice + yaw-align/forward DiffDrive MPPI (no strafe).
#       # Mirrors the real-Go2W profile shipped 2026-05-02.
set -u -o pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo1_go2_real.xml"

# has_wheels:=false propagates to base launch → skips wheel_velocity_controller
# spawn + go2w_hybrid_cmd_router + switches CHAMP stand-up preset to "go2".
# two_way_drive:=false disables FAR reverse mode — CHAMP's go2 preset doesn't
# have a validated reverse-walking gait, so FAR commanding REVERSE leaves the
# robot stuck (see docs/claude/sim_comparison.md for CHAMP-vs-MuJoCo notes).
exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=96" \
  "has_wheels:=false" \
  "two_way_drive:=false" \
  "$@"
