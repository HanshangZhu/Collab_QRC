#!/usr/bin/env bash
# Centralised vs decentralised CFPA2 exploration under inter-robot comms dropout.
#
# Matrix: MODES x DROPOUTS x NUM_TRIALS, each DURATION_SEC sim-seconds, on
# demo3_mixed (Go2W robot_a + Go2 robot_b), headless MuJoCo.
#
# Localization: real Fast-LIO. Trials where /odom/nav diverges (Fast-LIO blow-up)
# are auto-detected via odom_divergence_monitor and re-run up to MAX_RETRIES.
#
# Tunables (env):
#   MODES="centralised decentralised"   DROPOUTS="0.0 0.3 0.5 0.8"
#   NUM_TRIALS=10  DURATION_SEC=600  SCENE_AREA_M2=384.0
#   CFPA2_SUFFIX=_cpp  MAX_RETRIES=3  DIVERGENCE_BOUND_M=60  READY_TIMEOUT=120
#   OUT_DIR=/tmp/dropout_bench/<ts>
#
# Output layout: OUT_DIR/<mode>/d<pct>/trial_<n>/metrics.csv  (+ launch.log,
# divergence.json). Aggregated by scripts/bench/summarize_dropout_benchmark.py.
set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_DIR"

MODES="${MODES:-centralised decentralised}"
DROPOUTS="${DROPOUTS:-0.0 0.3 0.5 0.8}"
NUM_TRIALS="${NUM_TRIALS:-10}"
DURATION_SEC="${DURATION_SEC:-600}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
CFPA2_SUFFIX="${CFPA2_SUFFIX:-_cpp}"
MAX_RETRIES="${MAX_RETRIES:-3}"
DIVERGENCE_BOUND_M="${DIVERGENCE_BOUND_M:-60.0}"
READY_TIMEOUT="${READY_TIMEOUT:-120}"
MIN_PROGRESS_M="${MIN_PROGRESS_M:-3.0}"  # reject trials where neither robot moved (Nav2 didn't activate)
OUT_DIR="${OUT_DIR:-/tmp/dropout_bench/$(date +%Y%m%d_%H%M%S)}"
MUJOCO_LIB="${MUJOCO_LIB:-/home/hanszhu/miniforge3/envs/cmu_env/lib/python3.10/site-packages/mujoco}"
mkdir -p "$OUT_DIR"

# ROS setup scripts are not `set -u` safe; disable nounset only while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$MUJOCO_LIB"
# Disable DDS shared-memory (project convention): SHM ports accumulate across
# the many sequential launches in a sweep and degrade Nav2 controller_server
# activation, biasing late trials. UDP-only keeps every trial's bringup clean.
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS_DIR/config/fastdds_no_shm.xml"

LAUNCH="ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py"
RELAY_NS="[robot_a, robot_b]"

dpct() { python3 -c "print(f'd{int(round(float(\"$1\")*100)):02d}')"; }
verdict_diverged() {  # $1 = json path -> echoes "true"/"false"
    python3 -c "import json,sys
try:
    print(str(json.load(open(sys.argv[1])).get('diverged', False)).lower())
except Exception:
    print('false')" "$1" 2>/dev/null
}

run_one_attempt() {  # $1=mode $2=drop $3=tdir $4=seed -> 0 valid, 1 diverged/failed
    local mode="$1" drop="$2" tdir="$3" seed="$4"
    "$WS_DIR/scripts/debug/kill_sim.sh" >/dev/null 2>&1 || true
    sleep 3
    rm -f "$tdir/divergence.json" "$tdir"/exploration_*.csv

    $LAUNCH gui:=false rviz:=false explore:=true exploration_planner:=cfpa2 \
        coordination_mode:="$mode" cfpa2_executable_suffix:="$CFPA2_SUFFIX" \
        comms_dropout:="$drop" dropout_seed:="$seed" \
        metrics_logger:=false cleanup_stale:=false \
        > "$tdir/launch.log" 2>&1 &
    local launch_pid=$!

    # Wait for stack readiness (goal topic appears).
    local ready=0 i
    for ((i=0; i<READY_TIMEOUT; i++)); do
        if ros2 topic list 2>/dev/null | grep -q "/robot_a/way_point_coord"; then ready=1; break; fi
        if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
        sleep 1
    done
    if [ "$ready" -ne 1 ]; then
        echo "    [!] stack not ready in ${READY_TIMEOUT}s"
        "$WS_DIR/scripts/debug/kill_sim.sh" >/dev/null 2>&1 || true
        kill "$launch_pid" 2>/dev/null || true
        return 1
    fi

    # Divergence monitor (background) for this trial.
    python3 "$WS_DIR/scripts/runtime/odom_divergence_monitor.py" --ros-args \
        -p use_sim_time:=true -p namespaces:="$RELAY_NS" \
        -p bound_m:="$DIVERGENCE_BOUND_M" -p out_file:="$tdir/divergence.json" \
        > "$tdir/divergence.log" 2>&1 &
    local mon_pid=$!

    # Metrics logger (foreground) in union mode for DURATION_SEC sim-seconds.
    timeout "$DURATION_SEC" ros2 run go2w_observability exploration_metrics_logger.py \
        --ros-args -p use_sim_time:=true -p global_coverage_source:=union \
        -p scene_area_m2:="$SCENE_AREA_M2" -p namespaces:="$RELAY_NS" \
        -p output_dir:="$tdir" -p experiment_name:=trial \
        -p enable_stop_trigger:=false \
        > "$tdir/metrics.log" 2>&1 || true

    kill "$mon_pid" 2>/dev/null || true
    "$WS_DIR/scripts/debug/kill_sim.sh" >/dev/null 2>&1 || true
    kill "$launch_pid" 2>/dev/null || true
    # kill_sim doesn't know about these benchmark-scaffolding nodes; without an
    # explicit kill they orphan and accumulate DDS participants across trials,
    # degrading Nav2 controller_server activation in later trials. (Do NOT run
    # two benchmark drivers on one machine — this pkill is process-wide.)
    pkill -9 -f comms_dropout_relay.py 2>/dev/null || true
    pkill -9 -f odom_divergence_monitor.py 2>/dev/null || true
    sleep 2

    if [ "$(verdict_diverged "$tdir/divergence.json")" = "true" ]; then
        return 1
    fi
    local csv
    csv="$(ls -t "$tdir"/exploration_trial_*.csv 2>/dev/null | head -1)"
    if [ -z "$csv" ] || [ ! -s "$csv" ]; then
        echo "    [!] no metrics CSV produced"
        return 1
    fi
    cp "$csv" "$tdir/metrics.csv"

    # Reject trials where Nav2 never activated: neither robot moved meaningfully.
    local maxtraj
    maxtraj="$(python3 -c "
import csv
rows=list(csv.DictReader(open('$tdir/metrics.csv')))
if not rows: print(0.0)
else:
    l=rows[-1]
    a=float(l.get('robot_a_trajectory_m',0) or 0); b=float(l.get('robot_b_trajectory_m',0) or 0)
    print(max(a,b))" 2>/dev/null)"
    if python3 -c "import sys; sys.exit(0 if float('${maxtraj:-0}') < $MIN_PROGRESS_M else 1)"; then
        echo "    [!] no-progress (max robot traj ${maxtraj} m < ${MIN_PROGRESS_M} m) -- Nav2 likely never activated"
        rm -f "$tdir/metrics.csv"
        return 1
    fi
    return 0
}

echo "=== dropout benchmark -> $OUT_DIR ==="
echo "MODES=[$MODES] DROPOUTS=[$DROPOUTS] NUM_TRIALS=$NUM_TRIALS DURATION_SEC=$DURATION_SEC"
trial_idx=0
for mode in $MODES; do
  for drop in $DROPOUTS; do
    dtag="$(dpct "$drop")"
    for ((n=1; n<=NUM_TRIALS; n++)); do
      trial_idx=$((trial_idx+1))
      tdir="$OUT_DIR/$mode/$dtag/trial_$n"
      mkdir -p "$tdir"
      ok=0
      for ((attempt=1; attempt<=MAX_RETRIES+1; attempt++)); do
        seed=$(( trial_idx * 100 + attempt ))
        echo "[$mode $dtag trial $n] attempt $attempt seed=$seed"
        if run_one_attempt "$mode" "$drop" "$tdir" "$seed"; then
          echo "    -> VALID"
          ok=1; break
        else
          echo "    -> diverged/failed; retrying"
          mv "$tdir" "${tdir}.bad_attempt${attempt}" 2>/dev/null || true
          mkdir -p "$tdir"
        fi
      done
      [ "$ok" -ne 1 ] && echo "    [!] $mode $dtag trial $n exhausted retries (no valid trial)"
    done
  done
done

echo "=== aggregating ==="
python3 "$WS_DIR/scripts/bench/summarize_dropout_benchmark.py" "$OUT_DIR" | tee "$OUT_DIR/summary.txt"
echo "=== done: $OUT_DIR ==="
