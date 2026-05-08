#!/usr/bin/env bash
# Runs door demo + physics checker in parallel.
# Checker output goes to a log file and is tailed to terminal.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHECKER_LOG="/tmp/door_task_checker_$(date +%Y%m%d_%H%M%S).log"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi

safe_source "/opt/ros/humble/setup.bash"
safe_source "${WS_DIR}/install/setup.bash"

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
fi

cleanup() {
  kill "$CHECKER_PID" 2>/dev/null || true
  echo ""
  echo "=== CHECKER LOG: ${CHECKER_LOG} ==="
  tail -25 "$CHECKER_LOG" 2>/dev/null || true
}
trap cleanup EXIT

# Start the checker BEFORE the demo so it catches all events.
# It will wait for topics to appear.
echo "Starting door_task_checker → ${CHECKER_LOG}"
python3 "${WS_DIR}/scripts/door_task_checker.py" > "$CHECKER_LOG" 2>&1 &
CHECKER_PID=$!

# Launch the demo (all args forwarded)
exec "${WS_DIR}/scripts/door_demo_mujoco.sh" "$@"
