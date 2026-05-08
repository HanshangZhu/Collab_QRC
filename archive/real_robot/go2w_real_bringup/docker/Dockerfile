# ===========================================================================
# Stage 1: Build the go2w_driver and message packages from source
# ===========================================================================
FROM ros:humble-ros-base AS builder

SHELL ["/bin/bash", "-c"]

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-xacro \
    nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ws/src

# Copy ONLY the packages we need (no Gazebo, no SLAM, no nav)
# -- Message definitions --
COPY src/unitree_go2w_ros2/src/unitree_go  unitree_go/
COPY src/unitree_go2w_ros2/src/unitree_api unitree_api/
COPY src/unitree_go2w_ros2/src/go2_interfaces go2_interfaces/
# -- Driver + description + bringup --
COPY src/unitree_go2w_ros2/src/go2w_driver go2w_driver/
COPY src/unitree_go2w_ros2/src/go2w_description go2w_description/
COPY src/unitree_go2w_ros2/src/go2w_bringup go2w_bringup/
COPY src/unitree_go2w_ros2/src/go2w_mock go2w_mock/
COPY src/unitree_go2w_ros2/src/go2_rviz go2_rviz/

# Build
WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ===========================================================================
# Stage 2: Lean runtime image
# ===========================================================================
FROM ros:humble-ros-base AS runtime

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rclcpp-components \
  && rm -rf /var/lib/apt/lists/*

# Copy the built workspace from builder
COPY --from=builder /ws/install /ws/install

# Copy CycloneDDS config
COPY docker/go2w_real/cyclonedds_wifi.xml /ws/cyclonedds_wifi.xml

# Environment setup
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///ws/cyclonedds_wifi.xml
ENV ROS_DOMAIN_ID=0

# Entrypoint: source ROS2 + workspace overlay
COPY docker/go2w_real/entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh
ENTRYPOINT ["/ws/entrypoint.sh"]

# Default: launch the driver
CMD ["ros2", "launch", "go2w_bringup", "go2w.launch.py"]
