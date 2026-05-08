#!/usr/bin/env bash
# Multi-trial headless benchmark for FAR nav + CFPA2 exploration.
#
# Runs N sequential trials of nav_test_mujoco_fastlio.launch.py with:
#   nav_backend:=far
#   explore:=true
#   gui:=false  rviz:=false
#   enable_wall_checker:=false      (we want to measure contact counts,
#                                    not abort on first hit)
#   session_duration_sec:=<DUR>     (graceful launch shutdown on timeout)
#
# Each trial writes a JSON report via session_reporter.py. At the end we
# parse all per-trial JSONs and print a combined per-trial table + totals.
#
# Env overrides:
#   NUM_TRIALS     number of trials (default 5)
#   DURATION_SEC   per-trial session duration (default 120)
#   OUT_DIR        output directory (default /tmp/far_bench/<ts>)
#   EXTRA_ARGS     extra `ros2 launch` args (space-separated, optional)
#
# Usage:
#   ./scripts/benchmark_far_nav.sh
#   NUM_TRIALS=3 DURATION_SEC=60 ./scripts/benchmark_far_nav.sh
set -u -o pipefail
# note: do NOT use -e — individual trials may fail; we want to keep going.

NUM_TRIALS="${NUM_TRIALS:-5}"
DURATION_SEC="${DURATION_SEC:-120}"
OUT_DIR="${OUT_DIR:-/tmp/far_bench/$(date +%Y%m%d_%H%M%S)}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
# Sim ground-truth observable area (denominator for coverage_ratio).
# 96 m² = inner room of vlm_exploration_scene_no_artifacts.xml.
SCENE_AREA_M2="${SCENE_AREA_M2:-96.0}"
# Pass threshold: coverage_ratio must meet or exceed this to PASS.
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
# SC-PGO from LRC stack — add ONLY the sc_pgo prefix (sourcing the whole
# LRC setup.bash shadows our go2_gazebo_sim package because LRC also ships
# one; `ros2 launch` then fails to find nav_test_mujoco_fastlio.launch.py).
# See docs/claude/slam_and_scenes.md and debug_notes.md.
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
echo "  FAR NAV BENCHMARK"
echo "  workspace    : ${WS_DIR}"
echo "  trials       : ${NUM_TRIALS}"
echo "  duration/run : ${DURATION_SEC} s"
echo "  scene area   : ${SCENE_AREA_M2} m² (pass ≥ $(awk "BEGIN{print ${COVERAGE_PASS_FRACTION}*100}")%)"
echo "  out dir      : ${OUT_DIR}"
echo "  extra args   : ${EXTRA_ARGS:-<none>}"
echo "================================================================"

cleanup_procs() {
  # Kill sim + nav + helper node processes. Incomplete cleanup between trials
  # leaves zombies in DDS discovery → next trial's spawners fail to find
  # controller_manager service → all controllers time out → robot never moves.
  # Anything linked to our install/ tree or /opt/ros/.../lib/ tree is fair game.
  pkill -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -f 'ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio' 2>/dev/null || true
  pkill -f 'cartographer_node' 2>/dev/null || true
  pkill -f 'far_planner' 2>/dev/null || true
  pkill -f 'fastlio_mapping' 2>/dev/null || true
  pkill -f 'sc_pgo_node' 2>/dev/null || true
  pkill -f 'session_reporter.py' 2>/dev/null || true
  pkill -f 'far_wall_checker.py' 2>/dev/null || true
  pkill -f 'far_debug_monitor' 2>/dev/null || true
  pkill -f '/home/hz/Collab_QRC/install/' 2>/dev/null || true
  pkill -f '/opt/ros/humble/lib/' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'mujoco_ros2_control/mujoco_ros2_control' 2>/dev/null || true
  pkill -9 -f 'ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio' 2>/dev/null || true
  pkill -9 -f 'cartographer_node' 2>/dev/null || true
  pkill -9 -f 'far_planner' 2>/dev/null || true
  pkill -9 -f '/home/hz/Collab_QRC/install/' 2>/dev/null || true
  pkill -9 -f '/opt/ros/humble/lib/' 2>/dev/null || true
  # Clear FastDDS shared-memory segments so the next trial starts with a
  # fresh DDS discovery namespace (stale semaphores confuse topic/service
  # discovery, breaking controller_manager lookup).
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

  cleanup_procs

  # Outer timeout in case the launch hangs past session_duration_sec.
  # Launch file shuts itself down when session_reporter exits, but add
  # a 45 s safety margin for CHAMP standup + carto init + cleanup.
  outer_timeout=$((DURATION_SEC + 45))

  set +e
  timeout --signal=SIGTERM --kill-after=5 "${outer_timeout}" \
    ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py \
      nav_backend:=far \
      explore:=true \
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

  echo "  exit : ${rc}"
  if [[ ! -f "${trial_json}" ]]; then
    echo "  WARN : no JSON produced — likely an early crash. See log."
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
import json
import os
import sys
from pathlib import Path

out = Path(sys.argv[1])
trials = sorted(out.glob("trial_*.json"))

SCENE_AREA = float(os.environ.get("SCENE_AREA_M2", "96.0"))
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

n_pass = 0
n_completed = 0
n_cov_pass = 0
n_zero_contacts = 0
sum_area = 0.0
sum_dist = 0.0
sum_ratio = 0.0
sum_t = 0.0
sum_contacts = 0
sum_pairs = 0
sum_trans_err = 0.0
sum_yaw_err = 0.0
n_drift = 0
worst_contact_trial = None
all_robot_hits: dict[str, int] = {}
all_wall_hits: dict[str, int] = {}
failure_reasons: dict[str, int] = {}

for name, d in rows:
    outcome = d.get("outcome", "?")
    t = d.get("elapsed_sec", 0.0)
    cov = d.get("coverage", {})
    prog = d.get("progress", {})
    saf = d.get("safety", {})
    slam = d.get("slam", {})
    area = cov.get("explored_area_m2", 0.0)
    # coverage_ratio: use natively stored value if present, else compute
    # from explored_area / SCENE_AREA (back-compat for older JSONs).
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
        outcome == "completed"
        and contacts == 0
        and not prog.get("tipped_over")
        and cov_ok
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

    mark = "✓" if passed else "✗"
    print(
        f"{name:<10} {mark:<5} {outcome:<10} {t:>6.1f} "
        f"{area:>9.2f} {ratio*100:>6.1f}% {dist:>7.2f} "
        f"{contacts:>9d} {drift_t_str:>8} {drift_y_str:>8}"
    )
    if outcome == "completed":
        n_completed += 1
    if cov_ok:
        n_cov_pass += 1
    if contacts == 0:
        n_zero_contacts += 1
    if passed:
        n_pass += 1
    sum_area += area
    sum_dist += dist
    sum_ratio += ratio
    sum_t += t
    sum_contacts += contacts
    sum_pairs += pairs
    if trans_peak is not None:
        sum_trans_err += trans_peak
        sum_yaw_err += yaw_peak
        n_drift += 1
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
print(f"avg explored area       : {sum_area/n:.2f} m²  "
      f"({sum_ratio/n*100:.1f}% of {SCENE_AREA:.0f} m² gt)")
print(f"avg distance            : {sum_dist/n:.2f} m")
print(f"total wall contacts     : {sum_contacts}")
print(f"total unique geom pairs : {sum_pairs}")
if n_drift:
    print(f"avg SLAM trans drift pk : {sum_trans_err/n_drift:.3f} m")
    print(f"avg SLAM yaw drift pk   : {sum_yaw_err/n_drift:.2f}°")
else:
    print("SLAM drift              : n/a (no trial had gt+slam samples)")
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
    "trials": [dict(trial=n, **d) for n, d in rows],
    "aggregate": {
        "n_trials": n,
        "avg_elapsed_sec": sum_t / n,
        "n_completed": n_completed,
        "n_coverage_pass": n_cov_pass,
        "n_zero_contacts": n_zero_contacts,
        "n_full_pass": n_pass,
        "avg_explored_area_m2": sum_area / n,
        "avg_coverage_ratio": sum_ratio / n,
        "avg_distance_m": sum_dist / n,
        "total_wall_contacts": sum_contacts,
        "total_unique_pairs_hit": sum_pairs,
        "avg_slam_trans_drift_peak_m": (
            sum_trans_err / n_drift if n_drift else None
        ),
        "avg_slam_yaw_drift_peak_deg": (
            sum_yaw_err / n_drift if n_drift else None
        ),
        "walls_most_hit": all_wall_hits,
        "robot_parts_most_hit": all_robot_hits,
        "failure_reasons": failure_reasons,
    },
}, indent=2))
print(f"\ncombined JSON           : {combined}")
PY
