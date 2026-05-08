#!/usr/bin/env bash
# Recommended full-workspace build: merged install layout fixes CMake find_package()
# for sibling packages (e.g. visibility_graph_msg → boundary_handler) under heavy parallelism.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
python3 scripts/ops/workspace_colcon_audit.py
# ROS setup.bash reads optional env vars; `set -u` makes those lookups fatal.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
exec colcon build --symlink-install --merge-install "$@"
