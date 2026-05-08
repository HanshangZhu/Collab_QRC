#!/usr/bin/env bash
# Launch nav test for pure Go2 (no wheels) on demo3_go2 scene
# (24×16m, 384 m²). Same stack as nav_test_go2.sh but larger scene —
# exercises FAR V-graph routing across the 4 quadrants instead of a
# single small room.
#
# Usage:
#   ./scripts/nav_test_go2_demo3.sh                       # headless (FAR default)
#   ./scripts/nav_test_go2_demo3.sh gui:=true             # with MuJoCo GUI
#   ./scripts/nav_test_go2_demo3.sh gui:=true rviz:=true  # + RViz
#
# Nav2 MPPI stack (added 2026-05-02 PM):
#   ./scripts/nav_test_go2_demo3.sh gui:=true rviz:=true nav_backend:=nav2_mppi
#   ./scripts/nav_test_go2_demo3.sh gui:=true rviz:=true nav_backend:=nav2_mppi \
#       holonomic_profile:=se2_holonomic
set -u -o pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_go2_real.xml"

exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=384" \
  "has_wheels:=false" \
  "two_way_drive:=false" \
  "$@"
 
