#!/usr/bin/env bash
# Publish the top-right-corner test goal used for FAR Planner tuning.
#
# The goal goes to /goal_pose (same topic RViz 2D Goal Pose uses), which
# rviz_goal_relay then forwards to /robot/way_point_coord → far_planner.
#
# Usage:
#   ./scripts/far_publish_corner_goal.sh                  # default (11.5, 3.5)
#   ./scripts/far_publish_corner_goal.sh 11.5 3.5         # explicit x y
#   GOAL_YAW=1.57 ./scripts/far_publish_corner_goal.sh    # with heading

set -euo pipefail

GOAL_X="${1:-4.5}"
GOAL_Y="${2:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"

# Compute quaternion from yaw (z,w only — x,y are 0 for planar rotation).
QZ=$(python3 -c "import math; print(math.sin(${GOAL_YAW}/2.0))")
QW=$(python3 -c "import math; print(math.cos(${GOAL_YAW}/2.0))")

echo "[far_publish_corner_goal] publishing to /goal_pose: x=${GOAL_X} y=${GOAL_Y} yaw=${GOAL_YAW}"

# NOTE: --once / -1 races with DDS late-joiner discovery — rviz_goal_relay
# often misses the single message even though it is subscribed. Publish
# 5 times at 5 Hz (1 second total) so every subscriber gets at least one.
exec ros2 topic pub -t 5 -r 5 /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}
  }
}"
