FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-colcon-mixin \
    python3-vcstool python3-rosdep \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-hardware-interface ros-humble-realtime-tools \
    ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-robot-localization \
    ros-humble-tf2-tools ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-vision-opencv ros-humble-image-transport \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-cartographer-ros \
    ros-humble-octomap-server \
    ros-humble-nav2-bringup \
    ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard \
    ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libosmesa6-dev \
    libeigen3-dev libboost-all-dev libgflags-dev libgoogle-glog-dev \
    libyaml-cpp-dev nlohmann-json3-dev libopencv-dev \
    git curl wget cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Ensure colcon/cmake see ROS 2 on CMAKE_PREFIX_PATH even if setup.bash was not sourced.
RUN touch /root/.bashrc && grep -q 'source /opt/ros/humble/setup.bash' /root/.bashrc \
    || echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# livox_ros_driver2 links liblivox_lidar_sdk_shared.so from Livox-SDK2 (vendored under src/vendor).
# Install to /usr/local so find_library(... /usr/local/lib) succeeds without extra env vars.
COPY src/vendor/Livox-SDK2 /tmp/livox-sdk2-src
RUN cmake -S /tmp/livox-sdk2-src -B /tmp/livox-sdk2-build -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/livox-sdk2-build --parallel "$(nproc)" \
    && cmake --install /tmp/livox-sdk2-build \
    && rm -rf /tmp/livox-sdk2-build /tmp/livox-sdk2-src

RUN pip3 install --no-cache-dir \
    "mujoco==3.6.0" \
    numpy scipy matplotlib \
    opencv-python-headless Pillow requests \
    python-dotenv flask streamlit \
    catkin_pkg "empy==3.3.4" lark typeguard setuptools pyyaml \
    ortools

# Software OpenGL — avoids X11 OpenGL issues inside Docker on Mac
ENV MUJOCO_GL=osmesa
