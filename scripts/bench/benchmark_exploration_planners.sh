#!/usr/bin/env bash
# Benchmark CFPA2, GBPlanner2, and MTARE/TARE under the same Nav2 MPPI executor.
#
# Default matrix:
#   envs     : demo3_mixed + generated rooms/corridors mazes
#   planners : cfpa2 gbplanner2 mtare
#   trials   : 3
#   duration : 600 sim-seconds
#
# External planner hooks:
#   GBPLANNER2_EXTERNAL_CMD  long-running command that publishes
#                            /robot_a/command/trajectory and /robot_b/command/trajectory
#   MTARE_EXTERNAL_CMD       long-running command that publishes PointStamped
#                            waypoints on MTARE_WAYPOINT_TOPIC_A/B
#
# Without those hooks, cfpa2 runs normally, gbplanner2 starts the built-in dual
# UAS/Docker wrapper + adapters, and mtare uses the vendored TARE planner if it
# is built, otherwise the local autonomous fallback.
#
# Trav-CNN data collection + retraining (opt-in):
#   COLLECT_TRAV_CORPUS=true   collect live elevation patches + GT labels each trial
#   TRAV_RETRAIN=true          merge corpus + retrain CNN after all trials
#   TRAV_WEIGHTS_OUT=path      output weights file (default: OUT_DIR/weights_bench_retrain.dat)
#   TRAV_PRETRAIN_WEIGHTS=path existing weights to init from + anti-forgetting mix
#
# Example (collect + retrain):
#   COLLECT_TRAV_CORPUS=true TRAV_RETRAIN=true \
#   TRAV_PRETRAIN_WEIGHTS=src/vendor/elevation_mapping_cupy/.../weights_pretrain.dat \
#   ./scripts/bench/benchmark_exploration_planners.sh
set -u -o pipefail

NUM_TRIALS="${NUM_TRIALS:-3}"
DURATION_SEC="${DURATION_SEC:-600}"
SESSION_TIME_SOURCE="${SESSION_TIME_SOURCE:-sim}"
OUT_DIR="${OUT_DIR:-/tmp/exploration_bench/$(date +%Y%m%d_%H%M%S)}"
PLANNERS="${PLANNERS:-cfpa2 gbplanner2 mtare}"
SCENE_FILTER="${SCENE_FILTER:-}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
GUI="${GUI:-false}"
RVIZ="${RVIZ:-false}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
GBPLANNER2_EXTERNAL_CMD="${GBPLANNER2_EXTERNAL_CMD:-}"
MTARE_EXTERNAL_CMD="${MTARE_EXTERNAL_CMD:-}"
MTARE_WAYPOINT_TOPIC_A="${MTARE_WAYPOINT_TOPIC_A:-/robot_a/mtare/way_point}"
MTARE_WAYPOINT_TOPIC_B="${MTARE_WAYPOINT_TOPIC_B:-/robot_b/mtare/way_point}"
ALLOW_MTARE_FALLBACK="${ALLOW_MTARE_FALLBACK:-false}"

# ── Trav-CNN corpus collection ─────────────────────────────────────────────────
COLLECT_TRAV_CORPUS="${COLLECT_TRAV_CORPUS:-false}"
TRAV_RETRAIN="${TRAV_RETRAIN:-false}"
TRAV_WEIGHTS_OUT="${TRAV_WEIGHTS_OUT:-}"   # filled in after OUT_DIR is finalised
TRAV_PRETRAIN_WEIGHTS="${TRAV_PRETRAIN_WEIGHTS:-}"
TRAV_PATCHES_PER_FRAME="${TRAV_PATCHES_PER_FRAME:-50}"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UAS_REPO_ROOT="${UAS_REPO_ROOT:-$HOME/Research/uas_deploy/unified_autonomy_stack}"
GBPLANNER_COMPOSE_FILE="${GBPLANNER_COMPOSE_FILE:-${WS_DIR}/scripts/sim/gbplanner3_mujoco/compose/docker-compose.collab_qrc_dual.yml}"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
MJK_DIR="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco"
GENERATED_DIR="${MJK_DIR}/generated"
MTARE_VENDOR_EXECUTABLE_CANDIDATES=(
  "${WS_DIR}/src/vendor/tare_planner/install/tare_planner/lib/tare_planner/tare_planner_node"
  "${WS_DIR}/src/vendor/tare_planner/install/lib/tare_planner/tare_planner_node"
)

safe_source() { set +u; source "$1"; set -u; }

find_mtare_vendor_executable() {
  local candidate
  for candidate in "${MTARE_VENDOR_EXECUTABLE_CANDIDATES[@]}"; do
    if [[ -x "${candidate}" && -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

if [[ "${SESSION_TIME_SOURCE}" != "sim" && "${SESSION_TIME_SOURCE}" != "wall" ]]; then
  echo "ERROR: SESSION_TIME_SOURCE must be 'sim' or 'wall' (got '${SESSION_TIME_SOURCE}')" >&2
  exit 1
fi

for planner in ${PLANNERS}; do
  case "${planner}" in
    cfpa2|gbplanner2|mtare) ;;
    gbplanner3)
      echo "ERROR: gbplanner3 is intentionally excluded from the formal benchmark." >&2
      echo "       Use exploration_planner:=gbplanner3 from nav_test_demo3_mixed.sh for manual smoke/debug only." >&2
      exit 2
      ;;
    *)
      echo "ERROR: unsupported benchmark planner '${planner}'. Allowed: cfpa2 gbplanner2 mtare" >&2
      exit 2
      ;;
  esac
done

MTARE_VENDOR_EXECUTABLE=""
if [[ " ${PLANNERS} " == *" mtare "* && -z "${MTARE_EXTERNAL_CMD}" ]]; then
  MTARE_VENDOR_EXECUTABLE="$(find_mtare_vendor_executable || true)"
  if [[ -z "${MTARE_VENDOR_EXECUTABLE}" && "${ALLOW_MTARE_FALLBACK}" != "true" ]]; then
    echo "ERROR: mtare benchmark requires a built vendor TARE executable." >&2
    echo "       Expected one of:" >&2
    printf '       - %s\n' "${MTARE_VENDOR_EXECUTABLE_CANDIDATES[@]}" >&2
    echo "       Build src/vendor/tare_planner, or set ALLOW_MTARE_FALLBACK=true for non-formal fallback debug." >&2
    exit 2
  fi
fi

mkdir -p "${OUT_DIR}" "${GENERATED_DIR}"
: "${TRAV_WEIGHTS_OUT:=${OUT_DIR}/weights_bench_retrain.dat}"
python3 "${WS_DIR}/scripts/bench/generate_exploration_mazes.py" \
  --output-dir "${GENERATED_DIR}" >/tmp/exploration_bench_generate_mazes.log

SCENES=(
  "demo3_mixed:${MJK_DIR}/demo3_mixed.xml"
  "maze_rooms_seed101:${GENERATED_DIR}/demo3_mixed_rooms_seed101.xml"
  "maze_corridors_seed202:${GENERATED_DIR}/demo3_mixed_corridors_seed202.xml"
)

if [[ -n "${SCENE_FILTER}" ]]; then
  FILTERED_SCENES=()
  for scene_entry in "${SCENES[@]}"; do
    env_name="${scene_entry%%:*}"
    if [[ " ${SCENE_FILTER} " == *" ${env_name} "* ]]; then
      FILTERED_SCENES+=("${scene_entry}")
    fi
  done
  if [[ "${#FILTERED_SCENES[@]}" -eq 0 ]]; then
    echo "ERROR: SCENE_FILTER matched no scenes: ${SCENE_FILTER}" >&2
    exit 1
  fi
  SCENES=("${FILTERED_SCENES[@]}")
fi

cleanup_procs() {
  if [[ -f "${GBPLANNER_COMPOSE_FILE}" ]] && command -v docker >/dev/null 2>&1; then
    UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
    COLLAB_QRC_ROOT="${WS_DIR}" \
    docker compose -f "${GBPLANNER_COMPOSE_FILE}" --profile launch down --remove-orphans >/tmp/exploration_bench_gbplanner_down.log 2>&1 || true
  fi
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed' 2>/dev/null || true
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'cfpa2_coordinator_node' 2>/dev/null || true
  pkill -f 'gbplanner_to_waypoint_adapter.py' 2>/dev/null || true
  pkill -f 'external_point_to_waypoint_coord_adapter.py' 2>/dev/null || true
  pkill -f 'tare_planner_node' 2>/dev/null || true
  pkill -f 'tare_waypoint_watchdog.py' 2>/dev/null || true
  pkill -f 'fastlio_mapping' 2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'exploration_metrics_logger.py' 2>/dev/null || true
  pkill -f 'trav_corpus_collector.py' 2>/dev/null || true
  pkill -f 'controller_server' 2>/dev/null || true
  pkill -f 'planner_server' 2>/dev/null || true
  pkill -f 'bt_navigator' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f 'fastlio_mapping' 2>/dev/null || true
  rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
  sleep 2
}

trap 'echo "[benchmark] interrupted, cleaning up"; cleanup_procs; exit 130' INT TERM

# ── GT label-map pre-generation ────────────────────────────────────────────────
# trav_corpus_collector.py needs a static label .npy for each scene. Pre-build
# any that are missing now (one-shot per scene, ~10 s each, skipped if cached).
if [[ "${COLLECT_TRAV_CORPUS}" == "true" ]]; then
  echo "pre-generating GT label maps (missing only)..."
  for scene_entry in "${SCENES[@]}"; do
    scene_path="${scene_entry#*:}"
    label_npy="${scene_path%.xml}_gtlabel.npy"
    if [[ ! -f "${label_npy}" ]]; then
      echo "  building: $(basename "${label_npy}")"
      python3 "${WS_DIR}/scripts/training/mujoco_static_label_map.py" "${scene_path}"
    else
      echo "  cached : $(basename "${label_npy}")"
    fi
  done
fi

cat >"${OUT_DIR}/benchmark_config.json" <<EOF
{
  "num_trials": ${NUM_TRIALS},
  "duration_sec": ${DURATION_SEC},
  "session_time_source": "${SESSION_TIME_SOURCE}",
  "planners": "$(printf '%s' "${PLANNERS}")",
  "scene_filter": "$(printf '%s' "${SCENE_FILTER}")",
  "scene_area_m2": ${SCENE_AREA_M2},
  "gui": "${GUI}",
  "rviz": "${RVIZ}",
  "extra_args": "${EXTRA_ARGS}",
  "gbplanner2_external_cmd": "$(printf '%s' "${GBPLANNER2_EXTERNAL_CMD}")",
  "mtare_external_cmd": "$(printf '%s' "${MTARE_EXTERNAL_CMD}")",
  "allow_mtare_fallback": "${ALLOW_MTARE_FALLBACK}",
  "mtare_vendor_executable": "$(printf '%s' "${MTARE_VENDOR_EXECUTABLE}")",
  "collect_trav_corpus": "${COLLECT_TRAV_CORPUS}",
  "trav_retrain": "${TRAV_RETRAIN}",
  "trav_weights_out": "$(printf '%s' "${TRAV_WEIGHTS_OUT}")",
  "trav_pretrain_weights": "$(printf '%s' "${TRAV_PRETRAIN_WEIGHTS}")"
}
EOF

echo "================================================================"
echo "  Exploration planner benchmark"
echo "  workspace    : ${WS_DIR}"
echo "  out dir      : ${OUT_DIR}"
echo "  trials       : ${NUM_TRIALS}"
if [[ "${SESSION_TIME_SOURCE}" == "sim" ]]; then
echo "  duration/run : ${DURATION_SEC} sim-s"
else
echo "  duration/run : ${DURATION_SEC} wall-s"
fi
echo "  time source  : ${SESSION_TIME_SOURCE}"
echo "  planners     : ${PLANNERS}"
if [[ -n "${SCENE_FILTER}" ]]; then
echo "  scene filter : ${SCENE_FILTER}"
fi
echo "  scenes       : ${#SCENES[@]}"
echo "================================================================"

validation_failed=0

if [[ " ${PLANNERS} " == *" gbplanner2 "* && -z "${GBPLANNER2_EXTERNAL_CMD}" ]]; then
  echo "  gbplanner2  : using built-in dual UAS/Docker wrapper"
fi

if [[ " ${PLANNERS} " == *" mtare "* && -z "${MTARE_EXTERNAL_CMD}" ]]; then
  if [[ -n "${MTARE_VENDOR_EXECUTABLE}" ]]; then
    echo "  mtare       : using vendor TARE/MTARE (${MTARE_VENDOR_EXECUTABLE})"
  else
    echo "  mtare       : ALLOW_MTARE_FALLBACK=true, using local autonomous fallback"
  fi
fi

if [[ "${COLLECT_TRAV_CORPUS}" == "true" ]]; then
  echo "  trav corpus : ENABLED (${TRAV_PATCHES_PER_FRAME} patches/frame × 2 robots)"
  if [[ "${TRAV_RETRAIN}" == "true" ]]; then
    echo "  trav retrain: ENABLED → ${TRAV_WEIGHTS_OUT}"
    [[ -n "${TRAV_PRETRAIN_WEIGHTS}" ]] && echo "  pretrain mix: ${TRAV_PRETRAIN_WEIGHTS}"
  fi
fi

for scene_entry in "${SCENES[@]}"; do
  env_name="${scene_entry%%:*}"
  scene_path="${scene_entry#*:}"
  if [[ ! -f "${scene_path}" ]]; then
    echo "ERROR: missing scene ${scene_path}" >&2
    exit 1
  fi

  for planner in ${PLANNERS}; do
    for trial in $(seq -w 1 "${NUM_TRIALS}"); do
      trial_dir="${OUT_DIR}/${env_name}/${planner}/trial_${trial}"
      mkdir -p "${trial_dir}"
      trial_log="${trial_dir}/launch.log"
      cat >"${trial_dir}/launch_args.txt" <<EOF
exploration_planner:=${planner}
mujoco_model_path:=${scene_path}
scene_area_m2:=${SCENE_AREA_M2}
session_duration_sec:=${DURATION_SEC}
session_time_source:=${SESSION_TIME_SOURCE}
session_output_dir:=${trial_dir}
metrics_output_dir:=${trial_dir}
experiment_name:=${env_name}_${planner}_trial_${trial}
nav_backend_a:=nav2_mppi
nav_backend_b:=nav2_mppi
holonomic_profile_a:=se2_holonomic
holonomic_profile_b:=se2_holonomic
record_nav_bags:=false
EOF

      echo
      echo "---------- ${env_name} / ${planner} / trial_${trial} ----------"
      echo "  scene: ${scene_path}"
      echo "  dir  : ${trial_dir}"

      cleanup_procs
      if [[ "${planner}" == "gbplanner2" && -z "${GBPLANNER2_EXTERNAL_CMD}" ]]; then
        echo "  preparing ${planner} UAS workspace before starting sim"
        if ! UAS_REPO_ROOT="${UAS_REPO_ROOT}" \
          "${WS_DIR}/scripts/sim/gbplanner3_mujoco/prepare_gbplanner_ref.sh" "${planner}" \
          >"${trial_dir}/gbplanner_prepare.log" 2>&1; then
          echo "  ERROR: ${planner} prepare failed; tail:"
          tail -40 "${trial_dir}/gbplanner_prepare.log" | sed 's/^/    /'
          echo "prepare_failed" >"${trial_dir}/exit_code.txt"
          continue
        fi
      fi
      outer_timeout=$((DURATION_SEC * 3 + 180))
      optional_launch_args=()
      if [[ -n "${GBPLANNER2_EXTERNAL_CMD}" ]]; then
        optional_launch_args+=("gbplanner2_external_cmd:=${GBPLANNER2_EXTERNAL_CMD}")
      fi
      if [[ -n "${MTARE_EXTERNAL_CMD}" ]]; then
        optional_launch_args+=("mtare_external_cmd:=${MTARE_EXTERNAL_CMD}")
      fi
      # ── corpus collector (opt-in) ──────────────────────────────────────────
      _corpus_pids=()
      if [[ "${COLLECT_TRAV_CORPUS}" == "true" ]]; then
        corpus_dir="${trial_dir}/corpus"
        mkdir -p "${corpus_dir}"
        scene_stem="$(basename "${scene_path%.xml}")"
        label_npy="${scene_path%.xml}_gtlabel.npy"
        for _ns in robot_a robot_b; do
          python3 "${WS_DIR}/scripts/bench/trav_corpus_collector.py" \
            --namespace "${_ns}" \
            --scene "${scene_stem}" \
            --static-label "${label_npy}" \
            --output "${corpus_dir}/${_ns}_${env_name}_trial_${trial}.npz" \
            --patches-per-frame "${TRAV_PATCHES_PER_FRAME}" \
            >>"${trial_log}" 2>&1 &
          _corpus_pids+=($!)
        done
        echo "  corpus : collecting (pids: ${_corpus_pids[*]}) → ${corpus_dir}"
      fi

      set +e
      timeout --signal=SIGTERM --kill-after=15 "${outer_timeout}" \
        "${WS_DIR}/scripts/launch/nav_test_demo3_mixed.sh" \
          gui:="${GUI}" \
          rviz:="${RVIZ}" \
          exploration_planner:="${planner}" \
          mujoco_model_path:="${scene_path}" \
          scene_area_m2:="${SCENE_AREA_M2}" \
          session_duration_sec:="${DURATION_SEC}" \
          session_time_source:="${SESSION_TIME_SOURCE}" \
          session_output_dir:="${trial_dir}" \
          metrics_output_dir:="${trial_dir}" \
          experiment_name:="${env_name}_${planner}_trial_${trial}" \
          nav_backend_a:=nav2_mppi \
          nav_backend_b:=nav2_mppi \
          holonomic_profile_a:=se2_holonomic \
          holonomic_profile_b:=se2_holonomic \
          record_nav_bags:=false \
          mtare_waypoint_topic_a:="${MTARE_WAYPOINT_TOPIC_A}" \
          mtare_waypoint_topic_b:="${MTARE_WAYPOINT_TOPIC_B}" \
          "${optional_launch_args[@]}" \
          ${EXTRA_ARGS} \
          >"${trial_log}" 2>&1
      rc=$?
      set -u
      echo "${rc}" >"${trial_dir}/exit_code.txt"
      echo "  exit: ${rc}"

      # flush corpus collectors (SIGTERM → clean npz write)
      if [[ "${COLLECT_TRAV_CORPUS}" == "true" && "${#_corpus_pids[@]}" -gt 0 ]]; then
        for _pid in "${_corpus_pids[@]}"; do
          kill -TERM "${_pid}" 2>/dev/null || true
        done
        wait "${_corpus_pids[@]}" 2>/dev/null || true
        echo "  corpus : flushed (${corpus_dir})"
      fi

      if [[ ! -f "${trial_dir}/robot_a.json" || ! -f "${trial_dir}/robot_b.json" ]]; then
        echo "  WARN: session JSON missing; tail of log:"
        tail -20 "${trial_log}" | sed 's/^/    /'
      fi
      if python3 "${WS_DIR}/scripts/bench/validate_exploration_trial.py" \
        "${trial_dir}" \
        --duration-sec "${DURATION_SEC}" \
        >"${trial_dir}/validation.log" 2>&1; then
        echo "  validation: ok"
      else
        validation_failed=1
        echo "  validation: FAILED; details:"
        sed 's/^/    /' "${trial_dir}/validation.log"
      fi
    done
  done
done

cleanup_procs

# ── Trav-CNN retraining from collected corpus ──────────────────────────────────
if [[ "${COLLECT_TRAV_CORPUS}" == "true" && "${TRAV_RETRAIN}" == "true" ]]; then
  echo ""
  echo "================================================================"
  echo "  Trav-CNN retraining from benchmark corpus"
  merged_npz="${OUT_DIR}/corpus_merged.npz"
  echo "  merging corpus → ${merged_npz}"
  python3 "${WS_DIR}/scripts/training/merge_trav_corpus.py" \
    "${OUT_DIR}" --output "${merged_npz}"
  retrain_args=(
    "${merged_npz}"
    "${TRAV_WEIGHTS_OUT}"
    --epochs 200
    --lethal-weight 3.0
    --label-smoothing 0.05
  )
  if [[ -n "${TRAV_PRETRAIN_WEIGHTS}" ]]; then
    retrain_args+=(--init-from "${TRAV_PRETRAIN_WEIGHTS}")
    retrain_args+=(--mix-pretrain "${TRAV_PRETRAIN_WEIGHTS}" --mix-ratio 0.30)
  fi
  echo "  training → ${TRAV_WEIGHTS_OUT}"
  python3 "${WS_DIR}/scripts/training/train_trav_filter.py" "${retrain_args[@]}"
  echo "  done. deploy: copy ${TRAV_WEIGHTS_OUT} to elevation_mapping_cupy weights dir"
  echo "================================================================"
fi

python3 "${WS_DIR}/scripts/bench/aggregate_exploration_benchmark.py" "${OUT_DIR}" || true
echo "Benchmark output: ${OUT_DIR}"

if [[ "${validation_failed}" -ne 0 ]]; then
  echo "ERROR: one or more benchmark trials failed validation" >&2
  exit 3
fi
