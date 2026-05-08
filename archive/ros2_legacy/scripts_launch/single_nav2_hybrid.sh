#!/usr/bin/env bash
# Single-robot ISOLATED nav launch for the B-route nav2_hybrid_astar planner.
#
# Same scene & supporting stack as single_astar.sh, but the global +
# smoother are nav2_smac_planner library calls (AStarAlgorithm<NodeHybrid>
# + Smoother) wrapped by our LifecycleNode. Use this when you want to
# benchmark nav2_hybrid_astar in isolation, free from any other agent's
# /robot_a / /robot_b dual-sim that might collide on cmd_vel ownership.
#
# Usage:
#   ./scripts/launch/single_nav2_hybrid.sh                                    # Go2W, demo3, headless
#   ./scripts/launch/single_nav2_hybrid.sh robot:=go2 scene:=demo3 gui:=true rviz:=true
#   ./scripts/launch/single_nav2_hybrid.sh explore:=false                     # manual RViz goal
#   ./scripts/launch/single_nav2_hybrid.sh astar_config:=/tmp/my_nav2.yaml    # override params
#   ./scripts/launch/single_nav2_hybrid.sh session_duration_sec:=120          # bench-style bounded run
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

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

# nav_backend:=nav2_hybrid_astar selects the B-route planner. Default scene
# is demo3 to match what astar/hybrid_astar are usually compared against.
exec ros2 launch go2_gazebo_sim single_astar_mujoco.launch.py \
  nav_backend:=nav2_hybrid_astar \
  scene:=demo3 \
  "$@"
