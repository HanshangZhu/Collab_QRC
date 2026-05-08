#!/usr/bin/env bash
# Multi-trial benchmark for pure Go2 (no wheels) on demo3_go2 (24×16m, 384 m²).
#
# Env-overridable: NUM_TRIALS (default 5), DURATION_SEC (default 180),
#                  OUT_DIR, SCENE_AREA_M2.
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_go2_real.xml"

export SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
export DURATION_SEC="${DURATION_SEC:-180}"
export OUT_DIR="${OUT_DIR:-/tmp/far_bench/go2_demo3_$(date +%Y%m%d_%H%M%S)}"
export EXTRA_ARGS="${EXTRA_ARGS:-} mujoco_model_path:=${SCENE} has_wheels:=false two_way_drive:=false"

exec "${WS_DIR}/scripts/bench/benchmark_fastlio.sh" "$@"
