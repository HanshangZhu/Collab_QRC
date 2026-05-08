#!/bin/bash
# real_autonomy_go2.sh — shim for Unitree Go2 (no-wheel).
# Identical to real_autonomy.sh but pins robot=go2. All other flags forwarded.
#
# Examples:
#   ./real_autonomy_go2.sh                         # Go2 + carto_l1 + nav2_mppi (default)
#   ./real_autonomy_go2.sh slam=fastlio_mid360 nav=far
#   ./real_autonomy_go2.sh nav=tare
#   ./real_autonomy_go2.sh nav=cfpa2               # legacy default_nav.py
#   ./real_autonomy_go2.sh manual=true             # RViz click-to-nav on Nav2
#   ./real_autonomy_go2.sh stop
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# "stop" doesn't need robot flag.
if [[ "${1:-}" == "stop" ]]; then
  exec "$SCRIPT_DIR/real_autonomy.sh" stop
fi

exec "$SCRIPT_DIR/real_autonomy.sh" robot=go2 "$@"
