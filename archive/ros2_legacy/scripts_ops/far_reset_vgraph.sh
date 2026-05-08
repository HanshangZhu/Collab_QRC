#!/usr/bin/env bash
# Reset FAR Planner's accumulated visibility graph without restarting
# the sim. Useful when planning time goes catastrophic because the graph
# has drifted into a tangled mess of stale nodes/edges.
#
# Usage:
#   ./scripts/far_reset_vgraph.sh

set -euo pipefail
echo "[far_reset_vgraph] publishing std_msgs/Empty on /reset_visibility_graph"
exec ros2 topic pub -t 3 -r 5 /reset_visibility_graph std_msgs/msg/Empty "{}"
