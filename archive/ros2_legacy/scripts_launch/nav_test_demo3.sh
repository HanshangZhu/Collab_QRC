#!/usr/bin/env bash
# Launch nav test with demo3 (24×16m scene, 384 m², 4 thematic quadrants).
#   - NE: demo1 layout translated (preserves 0.425m narrow corridor)
#   - NW: open area with pillars (info-gain test)
#   - SW: multi-room lab with 1m doors (small-room entry test)
#   - SE: zig-zag S-corridor (path-follower turn test)
# Uses Fast-LIO2 + MID-360 by default.
# Usage:
#   ./scripts/nav_test_demo3.sh                            # GUI + RViz
#   ./scripts/nav_test_demo3.sh gui:=false                 # headless
#   ./scripts/nav_test_demo3.sh enable_wall_checker:=true  # test mode
set -euo pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3.xml"

exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" "mujoco_model_path:=${SCENE}" "scene_area_m2:=384" "$@"
