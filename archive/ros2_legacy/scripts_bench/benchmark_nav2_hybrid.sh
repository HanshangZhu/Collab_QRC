#!/usr/bin/env bash
# Multi-trial headless benchmark for nav2_hybrid_astar nav + CFPA2 exploration.
#
# Runs N sequential trials of single_astar_mujoco.launch.py with:
#   nav_backend:=nav2_hybrid_astar
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
#   OUT_DIR        output directory (default /tmp/nav2_hybrid_bench/<ts>)
#   SCENE          scene name (default demo3)
#   ROBOT          go2w | go2 (default go2w)
#   SCENE_AREA_M2  ground-truth observable area (default 384 for demo3)
#   COVERAGE_PASS_FRACTION  PASS bar (default 0.90)
#   EXTRA_ARGS     extra `ros2 launch` args (space-separated, optional)
#
# Usage:
#   ./scripts/bench/benchmark_nav2_hybrid.sh
#   NUM_TRIALS=3 DURATION_SEC=60 ./scripts/bench/benchmark_nav2_hybrid.sh
#   SCENE=demo1 SCENE_AREA_M2=96 ./scripts/bench/benchmark_nav2_hybrid.sh
set -u -o pipefail

NUM_TRIALS="${NUM_TRIALS:-5}"
DURATION_SEC="${DURATION_SEC:-120}"
NAV_BACKEND="${NAV_BACKEND:-nav2_hybrid_astar}"
OUT_DIR="${OUT_DIR:-/tmp/nav2_hybrid_bench/$(date +%Y%m%d_%H%M%S)_${NAV_BACKEND}}"
SCENE="${SCENE:-demo3}"
ROBOT="${ROBOT:-go2w}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
# demo1=96, demo3≈384.
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
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

mkdir -p "${OUT_DIR}"
echo "================================================================"
echo "  NAV BENCHMARK (backend=${NAV_BACKEND})"
echo "  workspace    : ${WS_DIR}"
echo "  trials       : ${NUM_TRIALS}"
echo "  duration/run : ${DURATION_SEC} s"
echo "  scene        : ${SCENE} (area=${SCENE_AREA_M2} m²)"
echo "  robot        : ${ROBOT}"
echo "  pass         : coverage ≥ $(awk "BEGIN{print ${COVERAGE_PASS_FRACTION}*100}")% AND contacts==0 AND ¬tipped"
echo "  output dir   : ${OUT_DIR}"
echo "  extra args   : ${EXTRA_ARGS:-<none>}"
echo "================================================================"

cleanup_procs() {
  pkill -f 'ros2 launch go2_gazebo_sim single_astar_mujoco' 2>/dev/null || true
  pkill -f 'mujoco_ros2_control --ros-args -r __ns:=/robot ' 2>/dev/null || true
  pkill -f 'fastlio_mapping --ros-args -r __node:=slam_node -r __ns:=/robot ' 2>/dev/null || true
  pkill -f 'champ_base.*-r __ns:=/robot ' 2>/dev/null || true
  pkill -f 'cfpa2_coordinator_node' 2>/dev/null || true
  pkill -f 'octomap_server.*__ns:=/robot ' 2>/dev/null || true
  pkill -f 'nav2_hybrid_astar_nav_node' 2>/dev/null || true
  pkill -f 'twist_bridge.py|go2w_hybrid_cmd_router' 2>/dev/null || true
  pkill -f 'stand_up_slowly|state_estimation_node|quadruped_controller_node|ekf_node|mujoco_odom_bridge|mujoco_contact_node|robot_state_publisher --ros-args' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'ros2 launch go2_gazebo_sim single_astar_mujoco' 2>/dev/null || true
  pkill -9 -f 'nav2_hybrid_astar_nav_node|mujoco_ros2_control --ros-args -r __ns:=/robot |fastlio_mapping --ros-args -r __node:=slam_node -r __ns:=/robot |champ_base.*-r __ns:=/robot |cfpa2_coordinator_node|octomap_server.*__ns:=/robot ' 2>/dev/null || true
  sleep 1
}

# Pre-clean any orphans from previous runs.
cleanup_procs

for ((i=1; i<=NUM_TRIALS; i++)); do
  trial_json="${OUT_DIR}/trial_$(printf '%02d' $i).json"
  trial_log="${OUT_DIR}/trial_$(printf '%02d' $i).log"
  echo
  echo "--- trial ${i}/${NUM_TRIALS} ---"
  echo "  json : ${trial_json}"
  echo "  log  : ${trial_log}"

  # Outer timeout in case the launch hangs past session_duration_sec.
  outer_timeout=$((DURATION_SEC + 60))

  set +e
  timeout --signal=SIGTERM --kill-after=10 "${outer_timeout}" \
    ros2 launch go2_gazebo_sim single_astar_mujoco.launch.py \
      nav_backend:="${NAV_BACKEND}" \
      robot:="${ROBOT}" \
      scene:="${SCENE}" \
      explore:=true \
      gui:=false \
      rviz:=false \
      cleanup_stale:=false \
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
  else
    # Quick one-line summary
    python3 -c "
import json
try:
    d = json.load(open('${trial_json}'))
    cov_node = d.get('coverage', {})
    prog = d.get('progress', {})
    cov = cov_node.get('coverage_ratio_of_scene', d.get('coverage_ratio', 0.0)) * 100
    expl = cov_node.get('explored_area_m2', 0.0)
    dist = prog.get('distance_travelled_m', 0.0)
    tipped = prog.get('tipped_over', d.get('tipped', False))
    tip_t = prog.get('first_tip_t_sec', None)
    contacts = d.get('contacts', {}).get('total_events', d.get('total_contacts', 'n/a'))
    out = d.get('outcome', 'unknown')
    print(f'  outcome={out} cov={cov:.1f}% expl={expl:.1f}m^2 dist={dist:.1f}m tipped={tipped}'
          + (f' (t={tip_t:.1f}s)' if (tipped and tip_t) else '')
          + f' contacts={contacts}')
except Exception as e:
    print(f'  WARN : json parse failed: {e}')
" || true
  fi

  # Forced cleanup between trials (preflight in launch should also catch).
  cleanup_procs
done

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
all_robot_hits = {}
all_wall_hits = {}
failure_reasons = {}

for name, d in rows:
    cov_node = d.get("coverage", {})
    prog_node = d.get("progress", {})
    safety_node = d.get("safety", {})
    slam_node = d.get("slam", {})

    completed = (d.get("outcome") == "completed")
    cov_ratio = float(cov_node.get("coverage_ratio_of_scene", 0.0))
    expl_m2 = float(cov_node.get("explored_area_m2", 0.0))
    dist_m = float(prog_node.get("distance_travelled_m", 0.0))
    elapsed = float(d.get("elapsed_sec", 0.0))
    wall_c = int(safety_node.get("wall_contact_count", 0))
    obs_c  = int(safety_node.get("obstacle_contact_count", 0))
    contacts = wall_c + obs_c
    pairs = len(safety_node.get("hit_obstacles", {})) + len(safety_node.get("hit_walls", {}))
    drift_m = slam_node.get("trans_error_peak_m")
    drift_deg = slam_node.get("yaw_error_peak_deg")
    tipped = bool(prog_node.get("tipped_over", False))
    tip_t = prog_node.get("first_tip_t_sec")

    cov_pct = cov_ratio * 100
    cov_ok = cov_ratio >= PASS_FRAC
    contacts_ok = contacts == 0
    not_tipped = not tipped
    full_pass = completed and cov_ok and contacts_ok and not_tipped

    if completed:    n_completed += 1
    if cov_ok:       n_cov_pass += 1
    if contacts_ok:  n_zero_contacts += 1
    if full_pass:    n_pass += 1

    sum_area += expl_m2
    sum_dist += dist_m
    sum_ratio += cov_ratio
    sum_t += elapsed
    sum_contacts += contacts
    sum_pairs += pairs

    if drift_m is not None and drift_deg is not None:
        sum_trans_err += float(drift_m)
        sum_yaw_err += float(drift_deg)
        n_drift += 1

    if worst_contact_trial is None or contacts > worst_contact_trial[1]:
        worst_contact_trial = (name, contacts)

    for w, c in (safety_node.get("hit_obstacles", {}) or {}).items():
        all_wall_hits[w] = all_wall_hits.get(w, 0) + int(c)
    for w, c in (safety_node.get("hit_walls", {}) or {}).items():
        all_wall_hits[w] = all_wall_hits.get(w, 0) + int(c)
    for r, c in (safety_node.get("hit_robot_parts", {}) or {}).items():
        all_robot_hits[r] = all_robot_hits.get(r, 0) + int(c)

    if not full_pass:
        why = []
        if not completed:   why.append("incomplete")
        if not cov_ok:      why.append(f"cov<{int(PASS_FRAC*100)}%")
        if not contacts_ok: why.append(f"contacts={contacts}")
        if not not_tipped:
            why.append(f"tipped@t={tip_t:.1f}s" if tip_t else "tipped")
        failure_reasons[name] = ",".join(why)

    outcome = "completed" if completed else "timed_out"
    print(
        f"{name:<10} {'YES' if full_pass else 'no':<5} {outcome:<10} "
        f"{elapsed:>6.1f} {expl_m2:>9.2f} {cov_pct:>6.1f}% "
        f"{dist_m:>7.2f} {contacts:>9d} "
        f"{(drift_m if drift_m is not None else 0.0):>8.3f} "
        f"{(drift_deg if drift_deg is not None else 0.0):>8.2f}"
    )

n = len(rows)
print("-" * 100)
print(f"trials                  : {n}")
print(f"FULL PASS (all gates)   : {n_pass}/{n}")
print(f"  - completed           : {n_completed}/{n}")
print(f"  - coverage ≥ {int(PASS_FRAC*100)}%      : {n_cov_pass}/{n}")
print(f"  - 0 contacts          : {n_zero_contacts}/{n}")
print(f"avg elapsed             : {sum_t/n:.1f} s")
print(f"avg explored area       : {sum_area/n:.2f} m²")
print(f"avg coverage ratio      : {sum_ratio/n*100:.1f}%")
print(f"avg distance traveled   : {sum_dist/n:.2f} m")
print(f"total wall contacts     : {sum_contacts}")
print(f"total unique geom pairs : {sum_pairs}")
if n_drift:
    print(f"avg SLAM trans drift pk : {sum_trans_err/n_drift:.3f} m")
    print(f"avg SLAM yaw drift pk   : {sum_yaw_err/n_drift:.2f}°")
else:
    print("SLAM drift              : n/a")
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
    },
}, indent=2))
print(f"\ncombined JSON           : {combined}")
PY

echo
echo "Reports & logs saved under: ${OUT_DIR}"
