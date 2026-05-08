#!/usr/bin/env bash
# replay_bag.sh — Offline replay of a real-robot rosbag for diagnosis.
#
# Usage:
#   ./scripts/real/replay_bag.sh <BAG_DIR> [rate=1.0] [rviz=true|false] [loop=true|false] [start=SEC]
#
# Examples:
#   ./scripts/real/replay_bag.sh bags/realrun_go2w_fastlio_mid360_nav2_mppi_20260501_143055
#   ./scripts/real/replay_bag.sh bags/realrun_*_20260501_143055 rate=2.0 loop=true
#   ./scripts/real/replay_bag.sh bags/realrun_*_20260501_143055 rviz=false
#
# Notes:
#   - Plays with --clock so any nodes you launch alongside (with
#     `use_sim_time:=true`) follow the recorded clock.
#   - RViz is launched with use_sim_time:=true and the autonomy.rviz
#     config used during real runs. Set rviz=false if you only want the
#     bag stream (e.g. for `ros2 topic hz` inspection).
#   - SIGINT / Ctrl+C cleans up RViz before exit.
#   - Pass `start=SEC` to skip ahead by N seconds (rosbag2 --start-offset).

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Args ─────────────────────────────────────────────────────────────
if [[ $# -lt 1 || "$1" == "-h" || "$1" == "--help" ]]; then
  sed -n '2,18p' "$0"
  exit 0
fi
BAG="$1"; shift || true

RATE="1.0"
USE_RVIZ="true"
LOOP="false"
START_OFFSET="0"
RVIZ_CONFIG="${REPO_ROOT}/src/go2w/go2w_real_bringup/config/rviz/autonomy.rviz"

for arg in "$@"; do
  case "$arg" in
    rate=*)        RATE="${arg#rate=}" ;;
    rviz=*)        USE_RVIZ="${arg#rviz=}" ;;
    loop=*)        LOOP="${arg#loop=}" ;;
    start=*)       START_OFFSET="${arg#start=}" ;;
    rviz_config=*) RVIZ_CONFIG="${arg#rviz_config=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$USE_RVIZ" in true|false) ;; *) echo "ERROR: rviz must be true|false" >&2; exit 1 ;; esac
case "$LOOP"     in true|false) ;; *) echo "ERROR: loop must be true|false" >&2; exit 1 ;; esac

[[ -d "$BAG" ]] || { echo "ERROR: bag directory not found: $BAG" >&2; exit 1; }

# Accept either a rosbag2 dir directly, or a parent containing one.
if [[ ! -f "$BAG/metadata.yaml" ]]; then
  cand="$(find "$BAG" -maxdepth 2 -name metadata.yaml -print -quit 2>/dev/null)"
  if [[ -n "$cand" ]]; then
    BAG="$(dirname "$cand")"
  else
    echo "ERROR: no metadata.yaml under $BAG (not a finalized rosbag2 directory)." >&2
    echo "       Was the recording killed with SIGTERM/SIGKILL? Check record.log:" >&2
    [[ -f "$BAG/record.log" ]] && tail -20 "$BAG/record.log" >&2
    exit 1
  fi
fi

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Replay: $BAG"
echo "    rate=$RATE  rviz=$USE_RVIZ  loop=$LOOP  start_offset=${START_OFFSET}s"
if [[ -f "$BAG/manifest.txt" ]]; then
  echo "  ── manifest ──"
  sed 's/^/    /' "$BAG/manifest.txt"
fi
echo "################################################"
echo ""

# ── Source ROS ───────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── RViz (background) ────────────────────────────────────────────────
RVIZ_PID=""
if [[ "$USE_RVIZ" == "true" ]]; then
  if [[ ! -f "$RVIZ_CONFIG" ]]; then
    echo "WARN: RViz config not found ($RVIZ_CONFIG); starting RViz with no preset." >&2
    ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  else
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
  fi
  RVIZ_PID=$!
  echo "  RViz PID: $RVIZ_PID"
fi

cleanup_replay() {
  trap - INT TERM EXIT
  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
  fi
  pkill -9 -f "ros2 bag play" 2>/dev/null || true
  exit 0
}
trap cleanup_replay INT TERM

# ── Play ─────────────────────────────────────────────────────────────
PLAY_ARGS=( --clock --rate "$RATE" )
[[ "$LOOP" == "true" ]] && PLAY_ARGS+=( --loop )
[[ "$START_OFFSET" != "0" ]] && PLAY_ARGS+=( --start-offset "$START_OFFSET" )

ros2 bag play "$BAG" "${PLAY_ARGS[@]}"
EXIT_CODE=$?

cleanup_replay
exit "$EXIT_CODE"
