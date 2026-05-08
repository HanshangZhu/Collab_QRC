#!/usr/bin/env bash
# Launch nav test with demo2 (24×16m complex room, 384 m²).
# Uses Fast-LIO2 + MID-360 by default.
# Usage:
#   ./scripts/nav_test_demo2.sh                        # GUI + RViz
#   ./scripts/nav_test_demo2.sh gui:=false              # headless
#   ./scripts/nav_test_demo2.sh enable_wall_checker:=true  # test mode
set -euo pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo2.xml"

exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" "mujoco_model_path:=${SCENE}" "scene_area_m2:=384" "$@"
