#!/bin/bash
# dry_run_go2.sh — Go2 (no-wheel) real-robot DRY-RUN.
#
# Full stack (SLAM + CFPA2/FAR/TARE + mux + supervisor panic) runs as normal,
# but `cmd_vel_to_sport_bridge` is skipped — nothing reaches the Unitree
# sport API. The robot stands still. Use for:
#   - pre-flight nav sanity (watch /robot/cmd_vel + RViz markers)
#   - verifying frontier/goal publication without movement risk
#   - SLAM + Fast-LIO A/B without committing to a teleop session
#
# Usage (flags are passed through to real_autonomy.sh):
#   ./dry_run_go2.sh                               # carto_l1 + cfpa2
#   ./dry_run_go2.sh slam=fastlio_mid360 nav=far
#   ./dry_run_go2.sh nav=tare
#   ./dry_run_go2.sh stop
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [[ "${1:-}" == "stop" ]]; then
  exec "$SCRIPT_DIR/real_autonomy.sh" stop
fi

exec "$SCRIPT_DIR/real_autonomy.sh" robot=go2 execute=false "$@"
