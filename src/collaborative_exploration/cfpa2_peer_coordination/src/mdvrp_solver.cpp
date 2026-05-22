// mdvrp_solver.cpp — Min-max MDVRP implementation.

#include "cfpa2_peer_coordination/mdvrp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#ifdef CFPA2_PC_HAS_ORTOOLS
// OR-Tools includes. Wrapped in the #ifdef so the build still
// succeeds when OR-Tools is not installed — see CMakeLists.txt
// for the find_package logic that toggles the macro.
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#endif

namespace cfpa2_peer_coordination {

namespace {

// Greedy nearest-neighbour ordering for a vehicle's assigned cells,
// starting from the depot. Used by the heuristic fallback and as
// the in-route ordering inside `_heuristic_mdvrp`.
std::vector<int> nearest_neighbor_order(
    const std::vector<int> & assigned_cells,
    const std::vector<std::vector<int>> & distance_matrix,
    int depot_node)
{
  std::vector<int> route;
  std::vector<int> remaining = assigned_cells;
  int cur = depot_node;
  while (!remaining.empty()) {
    auto best_it = std::min_element(
        remaining.begin(), remaining.end(),
        [&](int a, int b) {
          return distance_matrix[cur][a] < distance_matrix[cur][b];
        });
    cur = *best_it;
    route.push_back(cur);
    remaining.erase(best_it);
  }
  return route;
}

// Heuristic MDVRP — direct port of _heuristic_mdvrp in the Python.
// Greedy load-balanced assignment + nearest-neighbour ordering within
// each vehicle's assigned set. Independent of OR-Tools; always
// available as a fallback.
std::unordered_map<int, std::vector<int>> heuristic_mdvrp(
    const std::vector<Point3> & exploring_cell_positions,
    const std::vector<Point3> & robot_positions,
    const std::vector<std::vector<int>> & distance_matrix)
{
  const int num_cells = static_cast<int>(exploring_cell_positions.size());
  const int num_vehicles = static_cast<int>(robot_positions.size());

  std::unordered_map<int, std::vector<int>> routes;
  for (int i = 0; i < num_vehicles; ++i) {
    routes[i] = {};
  }
  if (num_cells <= 0 || num_vehicles <= 0) {
    return routes;
  }

  // mean_cost ≈ average distance entry; load_penalty discourages
  // piling every cell onto the nearest robot.
  std::uint64_t sum = 0;
  for (const auto & row : distance_matrix) {
    for (const int v : row) {
      sum += static_cast<std::uint64_t>(v);
    }
  }
  const std::size_t cells_total =
      distance_matrix.size() * distance_matrix.size();
  const double mean_cost = std::max(
      1.0,
      cells_total == 0
          ? 0.0
          : static_cast<double>(sum) / static_cast<double>(cells_total));
  const double load_penalty = 0.15 * mean_cost;

  std::unordered_map<int, std::vector<int>> cell_assignments;
  for (int i = 0; i < num_vehicles; ++i) {
    cell_assignments[i] = {};
  }

  for (int cell_idx = 0; cell_idx < num_cells; ++cell_idx) {
    int best_robot = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int robot_idx = 0; robot_idx < num_vehicles; ++robot_idx) {
      const int depot_idx = num_cells + robot_idx;
      const int projected_load =
          static_cast<int>(cell_assignments[robot_idx].size());
      const double cost =
          static_cast<double>(distance_matrix[depot_idx][cell_idx]) +
          (load_penalty * static_cast<double>(projected_load));
      if (cost < best_cost) {
        best_cost = cost;
        best_robot = robot_idx;
      }
    }
    cell_assignments[best_robot].push_back(cell_idx);
  }

  for (int robot_idx = 0; robot_idx < num_vehicles; ++robot_idx) {
    const int depot_idx = num_cells + robot_idx;
    routes[robot_idx] = nearest_neighbor_order(
        cell_assignments[robot_idx], distance_matrix, depot_idx);
  }
  return routes;
}

}  // namespace

std::vector<std::vector<int>> build_mdvrp_distance_matrix(
    const std::vector<Point3> & exploring_cell_positions,
    const std::vector<Point3> & robot_positions,
    double scale)
{
  std::vector<Point3> locations;
  locations.reserve(
      exploring_cell_positions.size() + robot_positions.size());
  locations.insert(locations.end(),
      exploring_cell_positions.begin(), exploring_cell_positions.end());
  locations.insert(locations.end(),
      robot_positions.begin(), robot_positions.end());

  std::vector<std::vector<int>> matrix;
  if (locations.empty()) {
    return matrix;
  }

  const double int_scale = std::max(1.0, scale);
  const std::size_t n = locations.size();
  matrix.assign(n, std::vector<int>(n, 0));

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      const double dx = locations[i][0] - locations[j][0];
      const double dy = locations[i][1] - locations[j][1];
      const double dz = locations[i][2] - locations[j][2];
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      matrix[i][j] = static_cast<int>(std::lround(d * int_scale));
    }
  }
  return matrix;
}

std::unordered_map<int, std::vector<int>> solve_mdvrp(
    const std::vector<Point3> & exploring_cell_positions,
    const std::vector<Point3> & robot_positions,
    const std::vector<std::vector<int>> & distance_matrix,
    double time_limit_sec,
    int span_cost_coefficient)
{
  const int num_cells = static_cast<int>(exploring_cell_positions.size());
  const int num_vehicles = static_cast<int>(robot_positions.size());
  const int num_locations = num_cells + num_vehicles;

  if (num_vehicles <= 0) {
    return {};
  }
  if (num_cells <= 0) {
    std::unordered_map<int, std::vector<int>> empty;
    for (int i = 0; i < num_vehicles; ++i) {
      empty[i] = {};
    }
    return empty;
  }
  if (static_cast<int>(distance_matrix.size()) != num_locations) {
    return {};
  }
  for (const auto & row : distance_matrix) {
    if (static_cast<int>(row.size()) != num_locations) {
      return {};
    }
  }

#ifdef CFPA2_PC_HAS_ORTOOLS
  using operations_research::DefaultRoutingSearchParameters;
  using operations_research::FirstSolutionStrategy;
  using operations_research::LocalSearchMetaheuristic;
  using operations_research::RoutingDimension;
  using operations_research::RoutingIndexManager;
  using operations_research::RoutingModel;
  using operations_research::RoutingSearchParameters;

  std::vector<RoutingIndexManager::NodeIndex> starts;
  std::vector<RoutingIndexManager::NodeIndex> ends;
  starts.reserve(num_vehicles);
  ends.reserve(num_vehicles);
  for (int i = 0; i < num_vehicles; ++i) {
    starts.emplace_back(num_cells + i);
    ends.emplace_back(num_cells + i);
  }

  RoutingIndexManager manager(num_locations, num_vehicles, starts, ends);
  RoutingModel routing(manager);

  const int transit_cb_index = routing.RegisterTransitCallback(
      [&manager, &distance_matrix](
          std::int64_t from_index, std::int64_t to_index) -> std::int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return distance_matrix[from_node][to_node];
      });
  routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_index);

  int max_arc = 0;
  for (const auto & row : distance_matrix) {
    for (const int v : row) {
      if (v > max_arc) max_arc = v;
    }
  }
  const std::int64_t max_route_cost = std::max<std::int64_t>(
      1,
      static_cast<std::int64_t>(max_arc) *
          std::max<std::int64_t>(2, num_cells + 1));
  routing.AddDimension(
      transit_cb_index,
      /*slack_max=*/0,
      max_route_cost,
      /*fix_start_cumul_to_zero=*/true,
      "Distance");
  RoutingDimension * distance_dim = routing.GetMutableDimension("Distance");
  distance_dim->SetGlobalSpanCostCoefficient(
      std::max(0, span_cost_coefficient));

  RoutingSearchParameters search_params = DefaultRoutingSearchParameters();
  search_params.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  search_params.set_local_search_metaheuristic(
      LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  const std::int64_t secs =
      std::max<std::int64_t>(1, static_cast<std::int64_t>(std::ceil(
                                    std::max(0.0, time_limit_sec))));
  search_params.mutable_time_limit()->set_seconds(secs);

  const operations_research::Assignment * solution =
      routing.SolveWithParameters(search_params);
  if (solution == nullptr) {
    return heuristic_mdvrp(
        exploring_cell_positions, robot_positions, distance_matrix);
  }

  std::unordered_map<int, std::vector<int>> routes;
  for (int vehicle_idx = 0; vehicle_idx < num_vehicles; ++vehicle_idx) {
    std::vector<int> route;
    std::int64_t index = routing.Start(vehicle_idx);
    while (!routing.IsEnd(index)) {
      const int node = manager.IndexToNode(index).value();
      if (node < num_cells) {
        route.push_back(node);
      }
      index = solution->Value(routing.NextVar(index));
    }
    routes[vehicle_idx] = std::move(route);
  }
  return routes;
#else
  (void)time_limit_sec;
  (void)span_cost_coefficient;
  return heuristic_mdvrp(
      exploring_cell_positions, robot_positions, distance_matrix);
#endif
}

}  // namespace cfpa2_peer_coordination