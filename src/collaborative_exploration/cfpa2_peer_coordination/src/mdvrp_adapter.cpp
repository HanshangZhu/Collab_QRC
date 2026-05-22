// mdvrp_adapter.cpp — Pure C++ wrapper around the MDVRP solver.

#include "cfpa2_peer_coordination/mdvrp_adapter.hpp"

#include <algorithm>
#include <cmath>


namespace cfpa2_peer_coordination {

namespace{

// Remove frontiers whose 2D distance to ANY robot is <= min_dist.
// Prevents "go to your current pose" assignments during stale-frontier
// moments. min_dist <= 0 disables the filter (returns input unchanged).
std::vector<Point3> filter_nearby_frontiers(
    const std::vector<Point3> & frontiers,
    const std::vector<Point3> & robot_positions,
    double min_dist)

{
  if (min_dist <= 0.0) {
    return frontiers;
  }
  std::vector<Point3> filtered;
  filtered.reserve(frontiers.size());
  for (const auto & frontier : frontiers) {
    bool too_close = false;
    for (const auto & robot : robot_positions) {
      if (distance_xy(frontier, robot) <= min_dist) {
        too_close = true;
        break;
      }
    }
    if (!too_close) {
      filtered.push_back(frontier);
    }
  }
  return filtered;
}

// Convert the upstream solver output (vehicle_index -> [cell_indices])
// to robot_id-keyed Point3 lists, using the caller's deterministic
// ordering of robot IDs.
Assignment route_indices_to_positions(
    const std::unordered_map<int, std::vector<int>> & routes,
    const std::vector<std::string> & robot_ids_sorted,
    const std::vector<Point3> & frontier_positions)
{
  Assignment assignments;
  for (const auto & robot_id : robot_ids_sorted) {
    assignments[robot_id] = {};
  }
  const int n_robots = static_cast<int>(robot_ids_sorted.size());
  const int n_frontiers = static_cast<int>(frontier_positions.size());
 
  // Defensive: solver should never emit out-of-range indices, but if
  // a future upstream bug does, skip rather than crash a long-running
  // node. Matches the Python adapter's behaviour.
  for (const auto & [robot_idx, route] : routes) {
    if (robot_idx < 0 || robot_idx >= n_robots) {
      continue;
    }
    const std::string & robot_id = robot_ids_sorted[robot_idx];
    for (const int frontier_idx : route) {
      if (frontier_idx >= 0 && frontier_idx < n_frontiers) {
        assignments[robot_id].push_back(frontier_positions[frontier_idx]);
      }
    }
  }
  return assignments;
}

}   // namespace

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

Point3 pose_msg_to_tuple(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {pose.position.x, pose.position.y, yaw};
}

Point3 point_msg_to_tuple(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y, point.z};
}

Assignment solve_frontier_assignment(
    const std::unordered_map<std::string, Point3> & robot_poses,
    const std::vector<Point3> & candidate_frontiers,
    double min_robot_to_frontier_dist,
    double time_limit_sec,
    int span_cost_coefficient)
{
  if (robot_poses.empty()) {
    return {};
  }

  std::vector<std::string> robot_ids_sorted;
  robot_ids_sorted.reserve(robot_poses.size());
  for (const auto & [robot_id, _pose] : robot_poses) {
    robot_ids_sorted.push_back(robot_id);
  }
  std::sort(robot_ids_sorted.begin(), robot_ids_sorted.end());

  std::vector<Point3> robot_positions;
  robot_positions.reserve(robot_ids_sorted.size());
  for (const auto & robot_id : robot_ids_sorted) {
    robot_positions.push_back(robot_poses.at(robot_id));
  }
 
  Assignment assignments;
  for (const auto & robot_id : robot_ids_sorted) {
    assignments[robot_id] = {};
  }

  // Sort frontiers lexicographically. std::array<double, 3> provides
  // operator< by element order, which matches the Python `key=lambda p:
  // (p[0], p[1], p[2])` sort key.
  std::vector<Point3> frontiers_sorted = candidate_frontiers;
  std::sort(frontiers_sorted.begin(), frontiers_sorted.end());
 
  const std::vector<Point3> filtered_frontiers = filter_nearby_frontiers(
      frontiers_sorted, robot_positions, min_robot_to_frontier_dist);

  if (filtered_frontiers.empty()) {
    return assignments;
  }

  if (robot_ids_sorted.size() == 1) {
    // Single robot — no optimisation problem to solve.
    assignments[robot_ids_sorted.front()] = filtered_frontiers;
    return assignments;
  }

  const std::vector<std::vector<int>> distance_matrix =
      build_mdvrp_distance_matrix(
          filtered_frontiers, robot_positions, kDistanceScale);
 
  const std::unordered_map<int, std::vector<int>> routes = solve_mdvrp(
      filtered_frontiers, robot_positions, distance_matrix,
      time_limit_sec, span_cost_coefficient);
 
  return route_indices_to_positions(
      routes, robot_ids_sorted, filtered_frontiers);
} 

}   // namespace cfpa2_peer_coordination