// mdvrp_solver.hpp — Min-max MDVRP helper functions.

#pragma once

#include <array>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace cfpa2_peer_coordination {
using Point3 = std::array<double, 3>;  // (x, y, yaw-or-z)

// Build a Euclidean distance matrix for [cells..., depots...] in the
// row/column ordering the OR-Tools routing model expects. Distances
// are multiplied by `scale` and rounded to int (OR-Tools requires
// integer arc costs). `scale` is clamped to a minimum of 1.0.
std::vector<std::vector<int>> build_mdvrp_distance_matrix(
    const std::vector<Point3> & exploring_cell_positions,
    const std::vector<Point3> & robot_positions,
    double scale = 100.0);

// Solve the min-max MDVRP via OR-Tools. Returns a mapping from robot_id// Solve the min-max MDVRP for the given cells/robots/distance matrix.
//
// Returns a map of `vehicle_index -> [cell_indices...]`. The vehicle
// index is the order of `robot_positions` (not robot ID). Cell indices
// are positions in `exploring_cell_positions`. Depot indices are NOT
// included in the returned route.
//
// Degenerate inputs return permissive defaults rather than throwing:
//   - num_vehicles <= 0            → empty map
//   - num_cells   <= 0            → {0: [], 1: [], ...}
//   - distance_matrix wrong shape → empty map
//
// time_limit_sec is rounded up to whole seconds because OR-Tools'
// Protobuf time_limit field is seconds-granular.
std::unordered_map<int, std::vector<int>> solve_mdvrp(
    const std::vector<Point3> & exploring_cell_positions,
    const std::vector<Point3> & robot_positions,
    const std::vector<std::vector<int>> & distance_matrix,
    double time_limit_sec = 1.0,
    int span_cost_coefficient = 100);

}  // namespace cfpa2_peer_coordination