// mdvrp_adapter.hpp — Pure C++ wrapper around the MDVRP solver.

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "cfpa2_peer_coordination/mdvrp_solver.hpp"  // Point3

namespace cfpa2_peer_coordination {

using Assignment = std::unordered_map<std::string, std::vector<Point3>>;

// Distance scaling: the upstream solver requires integer distances,
// so metres are multiplied by this factor before rounding. Matches
// the Python implementation (DISTANCE_SCALE = 100.0).
constexpr double kDistanceScale = 100.0;

// Assign candidate frontiers to robots via the upstream MDVRP solver.
//
//  robot_poses                   Mapping robot_id -> (x, y, yaw-or-z).
//                                Ordering is determined internally by
//                                lexicographic sort of the keys.
//  candidate_frontiers           (x, y, z) candidate frontier positions.
//  min_robot_to_frontier_dist    Filter threshold (metres); frontiers
//                                closer than this to ANY robot are
//                                dropped before solving.
//  time_limit_sec                Forwarded to the upstream solver.
//  span_cost_coefficient         Forwarded; higher = more aggressive
//                                load balancing.
//
// Returns: robot_id -> ordered list of assigned frontier positions.

Assignment solve_frontier_assignment(
    const std::unordered_map<std::string, Point3> & robot_poses,
    const std::vector<Point3> & candidate_frontiers,
    double min_robot_to_frontier_dist = 0.25,
    double time_limit_sec = 0.5,
    int span_cost_coefficient = 100);

// 2D Euclidean distance using x and y only.
double distance_xy(const Point3 & a, const Point3 & b);

// ROS message conversions. Kept on the adapter side so peer_coordinator
// callers don't need to know the tuple-convention used by the algorithm.
Point3 pose_msg_to_tuple(const geometry_msgs::msg::Pose & pose);
Point3 point_msg_to_tuple(const geometry_msgs::msg::Point & point);

} // namespace cfpa2_peer_coordination
