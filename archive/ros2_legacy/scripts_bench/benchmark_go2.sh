#!/usr/bin/env bash
# Multi-trial benchmark on pure Go2 (no wheels) with Livox MID-360 + FAR.
# Uses demo1_go2 (12×8m, 96 m² ground truth).
#
# Env-overridable:
#   NUM_TRIALS    (default 5)
#   DURATION_SEC  (default 120)
#   OUT_DIR       (default /tmp/far_bench/go2_$(date +%Y%m%d_%H%M%S))
#
# Example:
#   NUM_TRIALS=3 DURATION_SEC=180 ./scripts/benchmark_go2.sh
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo1_go2_real.xml"

export SCENE_AREA_M2="${SCENE_AREA_M2:-96.0}"
export DURATION_SEC="${DURATION_SEC:-120}"
export OUT_DIR="${OUT_DIR:-/tmp/far_bench/go2_$(date +%Y%m%d_%H%M%S)}"
# has_wheels:=false propagates through the launch chain; benchmark_fastlio.sh
# passes EXTRA_ARGS unchanged to ros2 launch. two_way_drive:=false disables
# FAR reverse (see scripts/nav_test_go2.sh for rationale).
export EXTRA_ARGS="${EXTRA_ARGS:-} mujoco_model_path:=${SCENE} has_wheels:=false two_way_drive:=false"

exec "${WS_DIR}/scripts/bench/benchmark_fastlio.sh" "$@"
