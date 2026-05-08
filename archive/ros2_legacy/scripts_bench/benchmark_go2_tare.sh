#!/usr/bin/env bash
# Multi-trial headless benchmark for Go2 + real CMU TARE → localPlanner
# (FAR unwired), with the sensor-derived waypoint watchdog armed.
#
# Runs N sequential trials of nav_test_go2_tare_real.launch.py with:
#   gui:=false  rviz:=false
#   enable_wall_checker:=false    (we measure contacts, not abort on first)
#   session_duration_sec:=<DUR>   (session_reporter drives graceful shutdown)
#   session_output_path:=<json>   (per-trial JSON report)
#   mujoco_model_path:=demo3_go2_real.xml  scene_area_m2:=384
#
# Defaults: 10 trials × 600 s (10 min) each. That's ~100–120 min wall
# clock total at demo3's ~0.5–0.6 RTF.
#
# Metrics (from session_reporter.py, same schema as benchmark_fastlio):
#   - outcome (completed / timed_out / ...)
#   - explored_area_m2 / coverage_ratio_of_scene
#   - distance_travelled_m
#   - wall_contact_count (MuJoCo live contacts — ground truth)
#   - unique_geom_pairs_hit
#   - slam drift peak (trans_m, yaw_deg) against ground-truth odom
#   - tipped_over flag
# PASS = completed && coverage ≥ 90 % && contacts == 0 && !tipped.
#
# Env overrides:
#   NUM_TRIALS      default 10
#   DURATION_SEC    default 600
#   OUT_DIR         default /tmp/tare_bench/<ts>
#   EXTRA_ARGS      extra launch args (space-separated)
#   SCENE_AREA_M2   default 384.0 (demo3_go2_real)
#   COVERAGE_PASS_FRACTION  default 0.90
#
# Usage:
#   ./scripts/bench/benchmark_go2_tare.sh
#   NUM_TRIALS=3 DURATION_SEC=180 ./scripts/bench/benchmark_go2_tare.sh
set -u -o pipefail
# no -e: individual trials may fail; keep going.

NUM_TRIALS="${NUM_TRIALS:-10}"
DURATION_SEC="${DURATION_SEC:-600}"
OUT_DIR="${OUT_DIR:-/tmp/tare_bench/$(date +%Y%m%d_%H%M%S)}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
COVERAGE_PASS_FRACTION="${COVERAGE_PASS_FRACTION:-0.90}"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"
# SC-PGO from LRC stack — only the sc_pgo prefix (avoid shadowing).
_SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${_SC_PGO_PREFIX}/share/sc_pgo" ]]; then
  export AMENT_PREFIX_PATH="${_SC_PGO_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${_SC_PGO_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${_SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export PATH="${_SC_PGO_PREFIX}/bin:${PATH}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

mkdir -p "${OUT_DIR}"
echo "================================================================"
echo "  TARE BENCHMARK (Go2 + TARE → localPlanner, FAR unwired)"
echo "  workspace    : ${WS_DIR}"
echo "  trials       : ${NUM_TRIALS}"
echo "  duration/run : ${DURATION_SEC} s  (total wall ≈ $((NUM_TRIALS * DURATION_SEC * 2 / 60)) min at ~0.5 RTF)"
echo "  scene area   : ${SCENE_AREA_M2} m² (pass ≥ $(awk "BEGIN{print ${COVERAGE_PASS_FRACTION}*100}")%)"
echo "  out dir      : ${OUT_DIR}"
echo "  extra args   : ${EXTRA_ARGS:-<none>}"
echo "================================================================"

cleanup_procs() {
  # Kill sim + nav + exploration + watchdog + bridge processes cleanly.
  # Order matters for DDS — kill launchers first, then children, then
  # clear shared-memory segments so the next trial has a fresh discovery
  # namespace (stale semaphores cause controller_manager lookup failures).
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_go2_tare_real' 2>/dev/null || true
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio' 2>/dev/null || true
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'tare_planner_node' 2>/dev/null || true
  pkill -f 'far_planner' 2>/dev/null || true
  pkill -f 'fastlio_mapping' 2>/dev/null || true
  pkill -f 'sc_pgo_node' 2>/dev/null || true
  pkill -f 'tare_waypoint_watchdog' 2>/dev/null || true
  pkill -f 'cloud_world_offset_bridge' 2>/dev/null || true
  pkill -f 'robust_controller_spawner' 2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'far_debug_monitor' 2>/dev/null || true
  pkill -f 'far_wall_checker' 2>/dev/null || true
  pkill -f '/home/hz/Collab_QRC/install/' 2>/dev/null || true
  pkill -f '/opt/ros/humble/lib/' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f 'tare_planner_node' 2>/dev/null || true
  pkill -9 -f 'far_planner' 2>/dev/null || true
  pkill -9 -f 'fastlio_mapping' 2>/dev/null || true
  pkill -9 -f '/home/hz/Collab_QRC/install/' 2>/dev/null || true
  pkill -9 -f '/opt/ros/humble/lib/' 2>/dev/null || true
  rm -f /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
  sleep 2
}

trap 'echo "[benchmark] INT — cleaning up"; cleanup_procs; exit 130' INT TERM

for i in $(seq 1 "${NUM_TRIALS}"); do
  trial_json="${OUT_DIR}/trial_${i}.json"
  trial_log="${OUT_DIR}/trial_${i}.log"

  echo
  echo "---------- TRIAL ${i}/${NUM_TRIALS} ----------"
  echo "  json : ${trial_json}"
  echo "  log  : ${trial_log}"
  echo "  start: $(date +%H:%M:%S)"

  cleanup_procs

  # Outer timeout in case the launch hangs past session_duration_sec.
  # RTF ≈ 0.5 on this laptop, so DURATION_SEC sim-seconds map to
  # ~2×DURATION_SEC wall-seconds. Add a 60 s margin for standup + cleanup.
  outer_timeout=$((DURATION_SEC * 2 + 60))

  set +e
  timeout --signal=SIGTERM --kill-after=10 "${outer_timeout}" \
    ros2 launch go2_gazebo_sim nav_test_go2_tare_real.launch.py \
      gui:=false \
      rviz:=false \
      enable_wall_checker:=false \
      session_duration_sec:="${DURATION_SEC}" \
      session_output_path:="${trial_json}" \
      scene_area_m2:="${SCENE_AREA_M2}" \
      ${EXTRA_ARGS} \
      >"${trial_log}" 2>&1
  rc=$?
  set -e

  echo "  exit : ${rc}   end: $(date +%H:%M:%S)"
  if [[ ! -f "${trial_json}" ]]; then
    echo "  WARN : no JSON produced — likely an early crash. Tail of log:"
    tail -12 "${trial_log}" | sed 's/^/    /'
  fi
done

cleanup_procs

echo
echo "================================================================"
echo "  SUMMARY"
echo "================================================================"

SCENE_AREA_M2="${SCENE_AREA_M2}" \
COVERAGE_PASS_FRACTION="${COVERAGE_PASS_FRACTION}" \
/usr/bin/python3 - "${OUT_DIR}" <<'PY'
import json, os, sys, statistics
from pathlib import Path

out = Path(sys.argv[1])
trials = sorted(out.glob("trial_*.json"))

SCENE_AREA = float(os.environ.get("SCENE_AREA_M2", "384.0"))
PASS_FRAC = float(os.environ.get("COVERAGE_PASS_FRACTION", "0.90"))

if not trials:
    print("NO TRIAL JSON FILES FOUND")
    print(f"check logs in {out}")
    sys.exit(1)

rows = []
for f in trials:
    try:
        rows.append((f.stem, json.loads(f.read_text())))
    except Exception as e:
        print(f"  parse error for {f.name}: {e}")

print(
    f"{'trial':<10} {'PASS':<5} {'outcome':<10} {'t s':>6} "
    f"{'expl m²':>9} {'cov %':>7} {'dist m':>7} "
    f"{'contacts':>9} {'drift m':>8} {'drift °':>8}"
)
print("-" * 100)

n_pass = n_completed = n_cov_pass = n_zero_contacts = 0
sum_area = sum_dist = sum_ratio = sum_t = 0.0
sum_contacts = sum_pairs = 0
cov_list: list[float] = []
dist_list: list[float] = []
time_list: list[float] = []
trans_peaks: list[float] = []
yaw_peaks: list[float] = []
worst_contact_trial = None
all_robot_hits: dict[str, int] = {}
all_wall_hits: dict[str, int] = {}
failure_reasons: dict[str, str] = {}

for name, d in rows:
    outcome = d.get("outcome", "?")
    t = d.get("elapsed_sec", 0.0)
    cov = d.get("coverage", {})
    prog = d.get("progress", {})
    saf = d.get("safety", {})
    slam = d.get("slam", {})
    area = cov.get("explored_area_m2", 0.0)
    ratio = cov.get("coverage_ratio_of_scene")
    if not ratio:
        ratio = area / SCENE_AREA if SCENE_AREA > 0 else 0.0
    cov_ok = ratio >= PASS_FRAC
    dist = prog.get("distance_travelled_m", 0.0)
    contacts = saf.get("wall_contact_count", 0)
    pairs = len(saf.get("unique_geom_pairs_hit", []))
    trans_peak = slam.get("trans_error_peak_m")
    yaw_peak = slam.get("yaw_error_peak_deg")
    drift_t_str = f"{trans_peak:.2f}" if trans_peak is not None else "n/a"
    drift_y_str = f"{yaw_peak:.1f}°" if yaw_peak is not None else "n/a"

    passed = (
        outcome == "completed" and contacts == 0
        and not prog.get("tipped_over") and cov_ok
    )
    reasons = []
    if outcome != "completed":
        reasons.append(f"outcome={outcome}")
    if contacts > 0:
        reasons.append(f"{contacts} wall hits")
    if prog.get("tipped_over"):
        reasons.append("tipped")
    if not cov_ok:
        reasons.append(f"cov={ratio*100:.1f}%<{PASS_FRAC*100:.0f}%")
    if reasons:
        failure_reasons[name] = "; ".join(reasons)

    mark = "PASS" if passed else "FAIL"
    print(
        f"{name:<10} {mark:<5} {outcome:<10} {t:>6.1f} "
        f"{area:>9.2f} {ratio*100:>6.1f}% {dist:>7.2f} "
        f"{contacts:>9d} {drift_t_str:>8} {drift_y_str:>8}"
    )
    if outcome == "completed": n_completed += 1
    if cov_ok: n_cov_pass += 1
    if contacts == 0: n_zero_contacts += 1
    if passed: n_pass += 1
    sum_area += area; sum_dist += dist; sum_ratio += ratio; sum_t += t
    sum_contacts += contacts; sum_pairs += pairs
    cov_list.append(ratio); dist_list.append(dist); time_list.append(t)
    if trans_peak is not None: trans_peaks.append(trans_peak)
    if yaw_peak is not None: yaw_peaks.append(yaw_peak)
    if worst_contact_trial is None or contacts > worst_contact_trial[1]:
        worst_contact_trial = (name, contacts)
    for k, v in saf.get("hit_walls", {}).items():
        all_wall_hits[k] = all_wall_hits.get(k, 0) + v
    for k, v in saf.get("hit_robot_parts", {}).items():
        all_robot_hits[k] = all_robot_hits.get(k, 0) + v

n = len(rows)
print("-" * 100)
print(f"trials                  : {n}")
print(f"time avg                : {sum_t/n:.1f} s")
print(f"outcome=completed       : {n_completed}/{n}")
print(f"coverage ≥{PASS_FRAC*100:.0f}%             : {n_cov_pass}/{n}")
print(f"zero wall contacts      : {n_zero_contacts}/{n}")
print(f"FULL PASS (all three)   : {n_pass}/{n}")
print()

def dist_line(label: str, xs: list[float], fmt: str = "{:.2f}"):
    if not xs:
        print(f"{label:<24}: n/a"); return
    mn, mx = min(xs), max(xs)
    md = statistics.median(xs)
    mean = statistics.fmean(xs)
    std = statistics.pstdev(xs) if len(xs) > 1 else 0.0
    print(f"{label:<24}: mean={fmt.format(mean)}  median={fmt.format(md)}  "
          f"σ={fmt.format(std)}  min={fmt.format(mn)}  max={fmt.format(mx)}")

dist_line("coverage ratio", cov_list, "{:.3f}")
dist_line("explored area (m²)", [r*SCENE_AREA for r in cov_list])
dist_line("distance travelled (m)", dist_list)
dist_line("trial duration (s)", time_list, "{:.1f}")
if trans_peaks: dist_line("SLAM peak trans drift (m)", trans_peaks, "{:.3f}")
if yaw_peaks:   dist_line("SLAM peak yaw drift (°)", yaw_peaks, "{:.2f}")
print(f"total wall contacts     : {sum_contacts}")
print(f"total unique geom pairs : {sum_pairs}")
if worst_contact_trial and worst_contact_trial[1]:
    print(f"worst contact trial     : {worst_contact_trial[0]} ({worst_contact_trial[1]} events)")
if all_wall_hits:
    top_walls = sorted(all_wall_hits.items(), key=lambda x: -x[1])[:6]
    print(f"walls most hit          : {top_walls}")
if all_robot_hits:
    top_robots = sorted(all_robot_hits.items(), key=lambda x: -x[1])[:6]
    print(f"robot parts most hit    : {top_robots}")
if failure_reasons:
    print()
    print("failure details:")
    for k, v in failure_reasons.items():
        print(f"  {k}: {v}")

combined = out / "summary.json"
combined.write_text(json.dumps({
    "scene_area_m2": SCENE_AREA,
    "coverage_pass_fraction": PASS_FRAC,
    "planner": "tare_real (TARE → localPlanner, FAR unwired) + waypoint watchdog",
    "trials": [dict(trial=n, **d) for n, d in rows],
    "aggregate": {
        "n_trials": n,
        "avg_elapsed_sec": sum_t / n,
        "n_completed": n_completed,
        "n_coverage_pass": n_cov_pass,
        "n_zero_contacts": n_zero_contacts,
        "n_full_pass": n_pass,
        "coverage_ratio_mean": statistics.fmean(cov_list) if cov_list else None,
        "coverage_ratio_median": statistics.median(cov_list) if cov_list else None,
        "coverage_ratio_stdev": statistics.pstdev(cov_list) if len(cov_list) > 1 else 0.0,
        "distance_m_mean": statistics.fmean(dist_list) if dist_list else None,
        "distance_m_stdev": statistics.pstdev(dist_list) if len(dist_list) > 1 else 0.0,
        "total_wall_contacts": sum_contacts,
        "total_unique_pairs_hit": sum_pairs,
        "slam_trans_drift_peak_mean_m": statistics.fmean(trans_peaks) if trans_peaks else None,
        "slam_yaw_drift_peak_mean_deg": statistics.fmean(yaw_peaks) if yaw_peaks else None,
        "walls_most_hit": all_wall_hits,
        "robot_parts_most_hit": all_robot_hits,
        "failure_reasons": failure_reasons,
    },
}, indent=2))
print(f"\ncombined JSON           : {combined}")
PY
