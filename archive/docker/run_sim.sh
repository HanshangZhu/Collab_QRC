#!/usr/bin/env bash
set -e

# Interactive MuJoCo needs GLFW + X11 (XQuartz on Mac). OSMesa is headless-only.
# Override: MUJOCO_GL=osmesa ./docker/run_sim.sh
MUJOCO_GL="${MUJOCO_GL:-glfw}"

# Start XQuartz and allow local X connections from Docker Desktop → host
open -a XQuartz 2>/dev/null || true
sleep 2
xhost +localhost 2>/dev/null || true
xhost + 127.0.0.1 2>/dev/null || true

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run --rm -it \
  -v "${REPO_ROOT}":/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=host.docker.internal:0 \
  -e "MUJOCO_GL=${MUJOCO_GL}" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/config/fastdds_no_shm.xml \
  collab-qrc-sim \
  bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec bash"
