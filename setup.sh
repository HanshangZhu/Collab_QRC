#!/usr/bin/env bash
# QRC_demo workspace setup for Ubuntu 22.04 + ROS 2 Humble.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh          # full install (ROS 2 + conda + deps + build)
#   ./setup.sh --deps   # system + ROS deps only, skip build
#   ./setup.sh --build  # skip installs, just build workspace
#
# Requires: Ubuntu 22.04 (Jammy), sudo access, internet.
#
# Sim-only by default: Livox-SDK2 is plain CMake (see COLCON_IGNORE there).
# livox_ros_driver2 is a normal ROS 2 package — build it after installing Livox-SDK2
# (Docker image installs to /usr/local; real robot: docs/claude/real_robot.md).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV_NAME="cmu_env"
PYTHON_VERSION="3.10"
# Upstream TARE-documented OR-Tools release; tare_planner links libortools.so
# directly from the unpacked tree.
ORTOOLS_TARBALL="or-tools_amd64_ubuntu-20.04_v9.3.10497.tar.gz"
ORTOOLS_URL="https://github.com/google/or-tools/releases/download/v9.3/${ORTOOLS_TARBALL}"
ORTOOLS_UNPACKED_DIR="or-tools_Ubuntu-20.04-64bit_v9.3.10497"

# ── Parse args ────────────────────────────────────────────────────────────────
INSTALL_DEPS=true
DO_BUILD=true
for arg in "$@"; do
  case "$arg" in
    --deps)  DO_BUILD=false ;;
    --build) INSTALL_DEPS=false ;;
    --help|-h)
      echo "Usage: ./setup.sh [--deps | --build]"
      echo "  (no args)  Full install: system deps + conda + ROS 2 + build"
      echo "  --deps     Install dependencies only, skip build"
      echo "  --build    Build workspace only, skip dependency install"
      exit 0
      ;;
  esac
done

log() { echo -e "\n\033[1;36m>>> $1\033[0m"; }

# ── 1. System packages ───────────────────────────────────────────────────────
if $INSTALL_DEPS; then

log "Installing system packages"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git curl wget gnupg lsb-release software-properties-common \
  python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libglfw3-dev libx11-dev xorg-dev libopencv-dev libpcl-dev libeigen3-dev \
  libboost-all-dev libgflags-dev libgoogle-glog-dev \
  libyaml-cpp-dev nlohmann-json3-dev \
  python3-numpy python3-scipy python3-pytest python3-transforms3d

# ── 2. ROS 2 Humble ──────────────────────────────────────────────────────────
if ! dpkg -s ros-humble-ros-base &>/dev/null; then
  log "Installing ROS 2 Humble"
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
fi

log "Installing ROS 2 Humble packages"
sudo apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  ros-humble-rviz2 \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  ros-humble-realtime-tools \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-urdf \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs \
  ros-humble-tf-transformations \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-perception-pcl \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-octomap-server \
  ros-humble-nav2-bringup \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-velodyne-gazebo-plugins \
  ros-humble-launch-xml \
  ros-humble-launch-yaml \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-fastrtps \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ecl-threads \
  ros-humble-rosidl-generator-dds-idl

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  log "Initializing rosdep"
  sudo rosdep init || true
fi
rosdep update --include-eol-distros || true

# ── 3. Conda / Micromamba ─────────────────────────────────────────────────────
if ! command -v micromamba &>/dev/null && ! command -v conda &>/dev/null; then
  log "Installing micromamba"
  # sudo required — /usr/local/bin is not user-writable on a fresh system.
  curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
    | sudo tar -xvj -C /usr/local/bin --strip-components=1 bin/micromamba
  eval "$(micromamba shell hook -s bash)"
  micromamba shell init -s bash -p ~/micromamba
  grep -qxF 'eval "$(micromamba shell hook -s bash)"' ~/.bashrc \
    || echo 'eval "$(micromamba shell hook -s bash)"' >> ~/.bashrc
fi

# Activate shell hook
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)" || true
fi

if ! micromamba env list 2>/dev/null | grep -q "${CONDA_ENV_NAME}" && \
   ! conda env list 2>/dev/null | grep -q "${CONDA_ENV_NAME}"; then
  log "Creating conda environment: ${CONDA_ENV_NAME}"
  micromamba create -n "${CONDA_ENV_NAME}" -c conda-forge \
    "python=${PYTHON_VERSION}" \
    numpy scipy matplotlib \
    -y
fi

log "Installing Python packages in ${CONDA_ENV_NAME}"
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate "${CONDA_ENV_NAME}"
elif command -v conda &>/dev/null; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV_NAME}"
fi

# ROS 2 Humble Python build toolchain. colcon drives message/idl codegen and
# package.xml parsing through these; empy must stay <4 (Humble breaks on 4.x).
pip install --quiet --upgrade \
  catkin_pkg \
  'empy==3.3.4' \
  lark \
  typeguard \
  setuptools pyyaml

# MuJoCo pinned to 3.6.0 to match the DFKI mujoco_ros2_control bridge ABI
# (see CLAUDE.md; 3.7.x ships incompatible layout).
pip install --quiet \
  'mujoco==3.6.0' \
  opencv-python-headless \
  Pillow

# VLM + dashboard runtime deps (door task :8080, vlm_explorer :8501).
pip install --quiet \
  requests \
  python-dotenv \
  flask \
  streamlit

# ── 4. Workspace vendor hygiene ──────────────────────────────────────────────
log "Applying vendor-tree fixes (COLCON_IGNORE markers, OR-Tools fetch, ...)"
cd "${SCRIPT_DIR}"

# autonomy_stack_go2/ duplicates every base_autonomy/* package from
# far_planner/ (byte-identical sources) plus a legacy far_planner_old.
# Ignore the whole tree; far_planner/ is canonical.
touch src/vendor/autonomy_stack_go2/COLCON_IGNORE

# mujoco_ros2_control examples (franka_mujoco, unitree_h1_mujoco,
# task_table_mujoco) pull deps we don't need (franka-description, etc).
touch src/vendor/mujoco_ros2_control/examples/COLCON_IGNORE

# Livox-SDK2 is a pure CMake lib (no package.xml) that colcon's CMake detector
# still picks up as package name livox_sdk2; ignore it — SDK is installed for
# livox_ros_driver2 via Dockerfile or workspace-local cmake (real_robot.md).
touch src/vendor/Livox-SDK2/COLCON_IGNORE

# ROS 1 / duplicate-name packages (also committed; touch is idempotent).
touch src/vendor/sc_pgo/fast_lio_sam/COLCON_IGNORE
touch src/collaborative_exploration/go2_tare_planner_ros2/generated/tare_planner/COLCON_IGNORE

# Bundled ROS 1 sub-workspace — reference only.
touch src/mtare_ros1_ws/COLCON_IGNORE 2>/dev/null || true

# tare_planner's install step symlinks data/; upstream ships it empty.
mkdir -p src/vendor/tare_planner/data

# tare_planner hardcodes a relative link to or-tools/lib/libortools.so.
# Upstream docs pin v9.3.10497 (Ubuntu 20.04 binary; works on 22.04).
if [ ! -f "src/vendor/tare_planner/or-tools/lib/libortools.so" ]; then
  log "Downloading OR-Tools v9.3.10497 for tare_planner (~100 MB)"
  TMP_TARBALL="$(mktemp --suffix=.tar.gz)"
  trap 'rm -f "${TMP_TARBALL}"' EXIT
  curl -fL -o "${TMP_TARBALL}" "${ORTOOLS_URL}"
  tar -xzf "${TMP_TARBALL}" -C src/vendor/tare_planner/
  mv "src/vendor/tare_planner/${ORTOOLS_UNPACKED_DIR}" \
     src/vendor/tare_planner/or-tools
  rm -f "${TMP_TARBALL}"
  trap - EXIT
fi

# ── 5. rosdep install ─────────────────────────────────────────────────────────
log "Installing workspace rosdep dependencies"
source /opt/ros/humble/setup.bash

# Skip keys for vendored non-rosdep packages and ROS 1 tokens that survive in
# vendored package.xml files (rosbag/roscpp/rospy/message_{generation,runtime}
# have no ROS 2 rosdep equivalents; apr is a ROS 1-era build dep).
rosdep install --from-paths src \
  --ignore-src -r -y \
  --skip-keys="mujoco_ros2_control livox_ros_driver2 apr rosbag roscpp rospy message_generation message_runtime" || true

fi  # end INSTALL_DEPS

# ── 6. Build workspace ───────────────────────────────────────────────────────
if $DO_BUILD; then

log "Building workspace"
cd "${SCRIPT_DIR}"

# Activate environment
if command -v micromamba &>/dev/null; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate "${CONDA_ENV_NAME}" 2>/dev/null || true
elif command -v conda &>/dev/null; then
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "${CONDA_ENV_NAME}" 2>/dev/null || true
fi
source /opt/ros/humble/setup.bash

# Belt-and-braces: re-apply vendor hygiene in --build-only runs too.
touch src/mtare_ros1_ws/COLCON_IGNORE 2>/dev/null || true
touch src/vendor/autonomy_stack_go2/COLCON_IGNORE 2>/dev/null || true
touch src/vendor/mujoco_ros2_control/examples/COLCON_IGNORE 2>/dev/null || true
touch src/vendor/Livox-SDK2/COLCON_IGNORE 2>/dev/null || true
touch src/vendor/sc_pgo/fast_lio_sam/COLCON_IGNORE 2>/dev/null || true
touch src/collaborative_exploration/go2_tare_planner_ros2/generated/tare_planner/COLCON_IGNORE 2>/dev/null || true
mkdir -p src/vendor/tare_planner/data

# Conda's libqhull_r (pulled by PCL find_package) transitively needs
# CXXABI_1.3.15 from libstdc++ 6.0.34 — only conda ships this; Ubuntu 22.04
# system libstdc++ is 6.0.30. Prepend conda's lib dir so ld resolves
# libstdc++.so.6 to conda's copy during linking.
if [ -n "${CONDA_PREFIX:-}" ]; then
  export LIBRARY_PATH="${CONDA_PREFIX}/lib:${LIBRARY_PATH:-}"
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(which python3)" \
  --parallel-workers "$(nproc)"

log "Build complete"
echo ""
echo "In every new shell:"
echo "  eval \"\$(micromamba shell hook -s bash)\" && micromamba activate ${CONDA_ENV_NAME}"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  export LIBRARY_PATH=\"\${CONDA_PREFIX}/lib:\${LIBRARY_PATH:-}\""
echo "  export LD_LIBRARY_PATH=\"\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH:-}\""
echo ""
echo "Launches:"
echo "  ./scripts/launch/door_demo_mujoco.sh      # dual-robot door task (needs .env.xai)"
echo "  ./scripts/launch/vlm_demo_mujoco.sh       # single-robot VLM (needs .env.xai)"
echo "  ./scripts/launch/nav_test_go2.sh          # Go2 CHAMP sim"
echo "  NUM_TRIALS=1 DURATION_SEC=30 OUT_DIR=/tmp/smoke \\"
echo "    ./scripts/bench/benchmark_far_nav.sh    # 30 s nav smoke test"

if [ ! -f ".env.xai" ]; then
  echo ""
  echo "NOTE: .env.xai not present at repo root."
  echo "      VLM demos (door task, vlm_demo) need XAI_API_KEY=... in this file."
fi

fi  # end DO_BUILD
