#!/usr/bin/env bash
# Multi-trial benchmark on demo3 (24×16m, 384 m², 4 thematic quadrants).
# Wraps benchmark_fastlio.sh with demo3-specific scene args.
#
# Env-overridable:
#   NUM_TRIALS      (default 5)
#   DURATION_SEC    (default 180 — bigger scene needs more time)
#   OUT_DIR         (default /tmp/far_bench/demo3_$(date +%Y%m%d_%H%M%S))
#
# Example:
#   NUM_TRIALS=10 DURATION_SEC=240 ./scripts/benchmark_demo3.sh
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3.xml"

export SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
export DURATION_SEC="${DURATION_SEC:-180}"
export OUT_DIR="${OUT_DIR:-/tmp/far_bench/demo3_$(date +%Y%m%d_%H%M%S)}"
export EXTRA_ARGS="${EXTRA_ARGS:-} mujoco_model_path:=${SCENE}"

exec "${WS_DIR}/scripts/bench/benchmark_fastlio.sh" "$@"
