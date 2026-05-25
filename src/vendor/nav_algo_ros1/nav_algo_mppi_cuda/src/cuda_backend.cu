// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// CudaBackend implementation. See cuda_backend.hpp for design rationale.
//
// Lifecycle hygiene (v2): all device buffers are RAII-owned by member
// DevicePtr<T> wrappers — ctor-half-failure is safe (member dtors unwind),
// and ~CudaBackend is a defaulted dtor (no manual cudaFree list to forget).
// Sticky CUDA error state is reset after every kernel launch.

#include "nav_algo_mppi_cuda/cuda_backend.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <cuda_runtime.h>

// Compile-time selector matching nav_algo_mppi_cuda/cuda_backend.hpp:
// pick whichever full-Optimizer header matches the current build (ROS 1
// nav_algo_core or ROS 2 nav2_mppi_controller_cuda). Both expose the same
// public Optimizer accessor surface (state(), control_sequence(), etc.).
#ifdef NAV_ALGO_MPPI_CUDA_USE_NAV2
  #include "nav2_mppi_controller/optimizer.hpp"
#else
  #include "nav_algo_core/mppi/optimizer.hpp"
#endif

#ifdef NAV_ALGO_CUDA_PROBE
#include <fstream>
namespace { std::atomic<uint64_t> g_optimize_calls{0}; }
#endif

namespace nav_algo_mppi_cuda
{
namespace
{

// Run a CUDA API call + throw on failure. For use inside CudaBackend
// methods (post-construction); ctor uses the throwIfCudaError directly
// via DevicePtr<T>'s constructor.
inline void cudaCheck(cudaError_t e, const char * msg)
{
  if (e != cudaSuccess) {
    throw std::runtime_error(
      std::string("CudaBackend ") + msg + ": " + cudaGetErrorString(e));
  }
}

// After a kernel launch, drain the sticky-error queue. Without this, a
// single failed launch poisons every subsequent CUDA API call in the
// process. Returns the rc for the caller to throw / log.
inline int finalizeLaunchRc(int rc, const char * which)
{
  if (rc != 0) {
    clearStickyCudaError();
    throw std::runtime_error(
      std::string("CudaBackend kernel ") + which + " failed rc=" + std::to_string(rc));
  }
  return rc;
}

}  // namespace

CudaBackend::CudaBackend(const CudaBackendConfig & cfg)
: cfg_(cfg),
  // Stream first so any subsequent allocation failures unwind cleanly.
  stream_{},
  // All device buffers sized to cfg_ max. Constructor of each DevicePtr<T>
  // throws on cudaMalloc failure — partial state cleans up via member dtors.
  d_traj_x_         (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_traj_y_         (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_traj_yaws_      (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_vx_       (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_vy_       (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_wz_       (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_cvx_      (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_cvy_      (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_state_cwz_      (static_cast<size_t>(cfg.batch_size) * cfg.time_steps),
  d_ctrl_vx_        (cfg.time_steps),
  d_ctrl_vy_        (cfg.time_steps),
  d_ctrl_wz_        (cfg.time_steps),
  d_costs_          (cfg.batch_size),
  d_softmax_        (cfg.batch_size),
  d_path_x_         (cfg.path_max_points),
  d_path_y_         (cfg.path_max_points),
  d_path_yaws_      (cfg.path_max_points),
  d_path_int_dist_  (cfg.path_max_points),
  d_path_pts_valid_ (cfg.path_max_points),
  d_costmap_        (cfg.costmap_max_cells),
  d_fp_x_           (cfg.footprint_max_n),
  d_fp_y_           (cfg.footprint_max_n),
  // Pinned host staging — currently used as a reserve for future
  // cudaMemcpyAsync overlap optimisations. v1 cuda_backend uses pageable
  // cudaMemcpy for simplicity; switching to async + pinned is a hot-path
  // optimisation that wants benchmarking on Orin first.
  h_costmap_stage_  (cfg.costmap_max_cells),
  h_state_stage_    (static_cast<size_t>(cfg.batch_size) * cfg.time_steps)
{
#ifdef NAV_ALGO_CUDA_PROBE
  // PROBE: ctor was reached. Append-only file with timestamps; gated by
  // build flag NAV_ALGO_CUDA_PROBE so production builds never touch disk.
  std::ofstream f("/tmp/cuda_backend_ctor", std::ios::app);
  auto t = std::chrono::system_clock::now().time_since_epoch();
  f << "ctor at " << std::chrono::duration_cast<std::chrono::milliseconds>(t).count()
    << "ms B=" << cfg.batch_size << " T=" << cfg.time_steps
    << " CM_max=" << cfg.costmap_max_cells << "\n";
#endif
}

void CudaBackend::setFootprint(
  const std::vector<float> & fp_x, const std::vector<float> & fp_y)
{
  if (fp_x.size() != fp_y.size() || fp_x.size() > cfg_.footprint_max_n) {
    throw std::runtime_error("CudaBackend::setFootprint: size mismatch or > footprint_max_n");
  }
  cudaCheck(cudaMemcpy(d_fp_x_.get(), fp_x.data(), fp_x.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "H2D fp_x");
  cudaCheck(cudaMemcpy(d_fp_y_.get(), fp_y.data(), fp_y.size() * sizeof(float),
                       cudaMemcpyHostToDevice), "H2D fp_y");
  fp_n_ = static_cast<unsigned int>(fp_x.size());
}

void CudaBackend::loadCriticParams(
  mppi::ParametersHandler * handler, const std::string & parent_name)
{
  if (handler == nullptr) {
    throw std::runtime_error("CudaBackend::loadCriticParams: handler is null");
  }

  // Each block mirrors the CPU critic's initialize() body: same param
  // name, same default value. Yaml edits to MPPIControllerROS.<Critic>.*
  // now propagate to the GPU path without a rebuild.
  {
    auto g = handler->getParamGetter(parent_name + ".ConstraintCritic");
    g(crit_.constraint_power, "cost_power", 1);
    g(crit_.constraint_weight, "cost_weight", 4.0f);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".ObstaclesCritic");
    g(crit_.obs_consider_footprint, "consider_footprint", false);
    g(crit_.obs_power, "cost_power", 1);
    g(crit_.obs_repulsion_weight, "repulsion_weight", 1.5f);
    g(crit_.obs_critical_weight, "critical_weight", 20.0f);
    g(crit_.obs_collision_cost, "collision_cost", 10000.0f);
    g(crit_.obs_collision_margin, "collision_margin_distance", 0.10f);
    g(crit_.obs_near_goal_distance, "near_goal_distance", 0.5f);
    // inflation_scale_factor + inflation_radius live under the
    // InflationLayer namespace; for sim we use the CPU CriticManager's
    // dynamic-load values, hardcoded here matching nav2_go2w_full_stack.yaml.
    crit_.obs_inflation_scale_factor = 5.0f;
    crit_.obs_inflation_radius       = 0.20f;
  }
  {
    auto g = handler->getParamGetter(parent_name + ".GoalCritic");
    g(crit_.goal_power, "cost_power", 1);
    g(crit_.goal_weight, "cost_weight", 5.0f);
    g(crit_.goal_threshold, "threshold_to_consider", 1.4f);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".GoalAngleCritic");
    g(crit_.goal_angle_power, "cost_power", 1);
    g(crit_.goal_angle_weight, "cost_weight", 3.0f);
    g(crit_.goal_angle_threshold, "threshold_to_consider", 0.5f);
    g(crit_.goal_angle_symmetric, "symmetric_yaw_tolerance", false);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".PathAlignCritic");
    g(crit_.path_align_power, "cost_power", 1);
    g(crit_.path_align_weight, "cost_weight", 10.0f);
    g(crit_.path_align_max_path_occupancy, "max_path_occupancy_ratio", 0.07f);
    g(crit_.path_align_offset_from_furthest, "offset_from_furthest", 20);
    g(crit_.path_align_trajectory_point_step, "trajectory_point_step", 4);
    g(crit_.path_align_threshold, "threshold_to_consider", 0.5f);
    g(crit_.path_align_use_orientations, "use_path_orientations", false);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".PathFollowCritic");
    g(crit_.path_follow_threshold, "threshold_to_consider", 1.4f);
    g(crit_.path_follow_offset_from_furthest, "offset_from_furthest", 6);
    g(crit_.path_follow_power, "cost_power", 1);
    g(crit_.path_follow_weight, "cost_weight", 5.0f);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".PathAngleCritic");
    g(crit_.path_angle_offset_from_furthest, "offset_from_furthest", 4);
    g(crit_.path_angle_power, "cost_power", 1);
    g(crit_.path_angle_weight, "cost_weight", 2.0f);
    g(crit_.path_angle_threshold, "threshold_to_consider", 0.5f);
    g(crit_.path_angle_max_angle_to_furthest, "max_angle_to_furthest", 1.2f);
    g(crit_.path_angle_forward_preference, "forward_preference", true);
  }
  {
    auto g = handler->getParamGetter(parent_name + ".PreferForwardCritic");
    g(crit_.prefer_forward_power, "cost_power", 1);
    g(crit_.prefer_forward_weight, "cost_weight", 5.0f);
    g(crit_.prefer_forward_threshold, "threshold_to_consider", 0.5f);
  }
}

void CudaBackend::optimize(mppi::Optimizer & opt)
{
#ifdef NAV_ALGO_CUDA_PROBE
  const uint64_t call_n = g_optimize_calls.fetch_add(1) + 1;
  if (call_n == 1) {
    std::ofstream f("/tmp/cuda_backend_optimize", std::ios::app);
    auto t = std::chrono::system_clock::now().time_since_epoch();
    f << "first optimize() at "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t).count() << "ms\n";
  }
  if (call_n % 50 == 0) {
    std::ofstream f("/tmp/cuda_backend_optimize", std::ios::app);
    f << "call_n=" << call_n << "\n";
  }
#endif

  auto & state    = opt.state();
  auto & ctrl     = opt.control_sequence();
  auto & trajs    = opt.generated_trajectories();
  auto & path     = opt.path();
  auto & costs    = opt.costs();
  auto & settings = opt.settings();
  const bool holonomic = opt.isHolonomicPublic();

  const unsigned int B = cfg_.batch_size;
  const unsigned int T = cfg_.time_steps;

  // Costmap pointer (live each cycle; map can be re-rolled by Nav2).
  auto * cm = opt.getCostmapForBackend();
  if (cm == nullptr) {
    throw std::runtime_error("CudaBackend::optimize: costmap is null");
  }
  const unsigned int cm_w = cm->getSizeInCellsX();
  const unsigned int cm_h = cm->getSizeInCellsY();
  if (static_cast<size_t>(cm_w) * cm_h > cfg_.costmap_max_cells) {
    throw std::runtime_error(
      "CudaBackend::optimize: costmap exceeds costmap_max_cells");
  }
  CostmapInfo cm_info{};
  cm_info.size_x     = cm_w;
  cm_info.size_y     = cm_h;
  cm_info.origin_x   = static_cast<float>(cm->getOriginX());
  cm_info.origin_y   = static_cast<float>(cm->getOriginY());
  cm_info.resolution = static_cast<float>(cm->getResolution());

  cudaCheck(cudaMemcpy(d_costmap_.get(), cm->getCharMap(),
                       static_cast<size_t>(cm_w) * cm_h * sizeof(uint8_t),
                       cudaMemcpyHostToDevice), "H2D costmap");

  // Path upload + path_pts_valid (now computed from costmap host-side,
  // matching utils::setPathCostsIfNotSet: a path point is valid iff its
  // cell cost < INSCRIBED_INFLATED_OBSTACLE (253). Replaces the v1
  // all-true placeholder).
  const unsigned int P = path.x.shape(0);
  if (P > cfg_.path_max_points) {
    throw std::runtime_error("CudaBackend::optimize: path > path_max_points");
  }
  std::vector<uint8_t> pts_valid(P, 1);  // declared at this scope so gates below can read it
  if (P > 0) {
    cudaCheck(cudaMemcpy(d_path_x_.get(),    path.x.data(),    P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.x");
    cudaCheck(cudaMemcpy(d_path_y_.get(),    path.y.data(),    P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.y");
    cudaCheck(cudaMemcpy(d_path_yaws_.get(), path.yaws.data(), P * sizeof(float), cudaMemcpyHostToDevice), "H2D path.yaws");
    std::vector<float> int_dist(P, 0.0f);
    const uint8_t * cm_host = cm->getCharMap();
    for (unsigned int i = 0; i < P; ++i) {
      if (i > 0) {
        const float dx = path.x(i) - path.x(i - 1);
        const float dy = path.y(i) - path.y(i - 1);
        int_dist[i] = int_dist[i - 1] + std::sqrt(dx * dx + dy * dy);
      }
      // Cell-cost check: out-of-bounds OR cost ≥ INSCRIBED_INFLATED_OBSTACLE
      // (253) marks the path point invalid so PathAlign's max_path_occupancy
      // gate can fire on heavily-blocked plans.
      const float wx = path.x(i);
      const float wy = path.y(i);
      pts_valid[i] = 0;
      if (wx >= cm_info.origin_x && wy >= cm_info.origin_y) {
        const unsigned int mx = static_cast<unsigned int>((wx - cm_info.origin_x) / cm_info.resolution);
        const unsigned int my = static_cast<unsigned int>((wy - cm_info.origin_y) / cm_info.resolution);
        if (mx < cm_info.size_x && my < cm_info.size_y) {
          pts_valid[i] = (cm_host[my * cm_info.size_x + mx] < 253) ? 1 : 0;
        }
      }
    }
    cudaCheck(cudaMemcpy(d_path_int_dist_.get(),  int_dist.data(),  P * sizeof(float),   cudaMemcpyHostToDevice), "H2D int_dist");
    cudaCheck(cudaMemcpy(d_path_pts_valid_.get(), pts_valid.data(), P * sizeof(uint8_t), cudaMemcpyHostToDevice), "H2D pts_valid");
  }

  // ── Host-side gating scalars (computed once per cycle; trivially cheap)
  // Mirrors the per-critic gate logic from nav_algo_core/src/mppi/critics/
  // *.cpp's score() bodies. All math is double-precision on host; gates
  // are pure boolean checks before kernel launches.
  const float robot_x = static_cast<float>(state.pose.pose.position.x);
  const float robot_y = static_cast<float>(state.pose.pose.position.y);
  float robot_yaw;
  {
    const auto & q = state.pose.pose.orientation;
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    robot_yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  // within_position_goal_tolerance(threshold) — Nav2 utils variant.
  // Empty path is treated as "within" (degenerate; CPU returns true too).
  double dist_to_goal_sq = 0.0;
  if (P > 0) {
    const float dx = robot_x - path.x(P - 1);
    const float dy = robot_y - path.y(P - 1);
    dist_to_goal_sq = static_cast<double>(dx) * dx + static_cast<double>(dy) * dy;
  }
  auto within_goal = [&](float threshold) -> bool {
    if (P == 0) return true;
    return dist_to_goal_sq < static_cast<double>(threshold) * threshold;
  };

  // NOTE: furthest_reached_path_point and the PathAlign/PathAngle gates that
  // depend on it are computed INSIDE the iteration loop, right after the
  // integrate kernel (block "1b" below) — they need the integrated trajectory
  // ENDPOINTS to faithfully mirror nav2 utils::findPathFurthestReachedPoint
  // (max over the bundle of each trajectory-end's closest path index). The
  // earlier "closest path point to the current robot pose" version pinned the
  // PathFollow carrot ~offset_from_furthest points ahead of the robot forever,
  // killing the speed feedback loop and capping vx at ~carrot_dist/horizon
  // (≈0.1 m/s) regardless of path length / costmap.

  // ObstaclesCritic's near_goal flag (controls whether repulsion runs).
  const bool near_goal = within_goal(crit_.obs_near_goal_distance);

  for (unsigned int it = 0; it < settings.iteration_count; ++it) {
    opt.generateNoisedTrajectoriesNoIntegrate();

    const size_t Nbt = static_cast<size_t>(B) * T * sizeof(float);
    cudaCheck(cudaMemcpy(d_state_vx_.get(),  state.vx.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.vx");
    cudaCheck(cudaMemcpy(d_state_wz_.get(),  state.wz.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.wz");
    cudaCheck(cudaMemcpy(d_state_cvx_.get(), state.cvx.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cvx");
    cudaCheck(cudaMemcpy(d_state_cwz_.get(), state.cwz.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cwz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(d_state_vy_.get(),  state.vy.data(),  Nbt, cudaMemcpyHostToDevice), "H2D state.vy");
      cudaCheck(cudaMemcpy(d_state_cvy_.get(), state.cvy.data(), Nbt, cudaMemcpyHostToDevice), "H2D state.cvy");
    }
    cudaCheck(cudaMemcpy(d_ctrl_vx_.get(), ctrl.vx.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.vx");
    cudaCheck(cudaMemcpy(d_ctrl_wz_.get(), ctrl.wz.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.wz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(d_ctrl_vy_.get(), ctrl.vy.data(), T * sizeof(float), cudaMemcpyHostToDevice), "H2D ctrl.vy");
    }

    // ── 1. Integrate kernel
    {
      IntegrateConfig icfg{};
      icfg.batch_size  = B;
      icfg.time_steps  = T;
      icfg.dt          = settings.model_dt;
      icfg.initial_x   = static_cast<float>(state.pose.pose.position.x);
      icfg.initial_y   = static_cast<float>(state.pose.pose.position.y);
      {
        const auto & q = state.pose.pose.orientation;
        const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        icfg.initial_yaw = std::atan2(siny_cosp, cosy_cosp);
      }
      icfg.holonomic = holonomic;
      finalizeLaunchRc(
        launchIntegrateStateVelocities(
          icfg, d_state_vx_, holonomic ? d_state_vy_.get() : nullptr, d_state_wz_,
          d_traj_x_, d_traj_y_, d_traj_yaws_),
        "integrate");
    }

    // ── 1b. furthest_reached_path_point — FAITHFUL port of nav2
    // utils::findPathFurthestReachedPoint: for each trajectory find the path
    // index closest to that trajectory's ENDPOINT, then take the MAX index
    // across the bundle. Computed here (after integrate) because it needs the
    // integrated traj endpoints; cudaMemcpy2D pulls the last-timestep column
    // (one float per trajectory, stride T). This is what advances the
    // PathFollow carrot as the bundle reaches further down the path, restoring
    // the speed feedback loop the pre-loop closest-to-robot version had killed.
    unsigned int furthest_reached = 0;
    if (P > 0) {
      std::vector<float> end_x(B), end_y(B);
      cudaCheck(cudaMemcpy2D(
        end_x.data(), sizeof(float),
        d_traj_x_.get() + (T - 1), static_cast<size_t>(T) * sizeof(float),
        sizeof(float), static_cast<size_t>(B), cudaMemcpyDeviceToHost),
        "D2H traj end_x");
      cudaCheck(cudaMemcpy2D(
        end_y.data(), sizeof(float),
        d_traj_y_.get() + (T - 1), static_cast<size_t>(T) * sizeof(float),
        sizeof(float), static_cast<size_t>(B), cudaMemcpyDeviceToHost),
        "D2H traj end_y");
      for (unsigned int b = 0; b < B; ++b) {
        const float ex = end_x[b];
        const float ey = end_y[b];
        unsigned int min_id = 0;
        double min_d2 = 1e30;
        for (unsigned int j = 0; j < P; ++j) {
          const float dx = ex - path.x(j);
          const float dy = ey - path.y(j);
          const double d2 = static_cast<double>(dx) * dx + static_cast<double>(dy) * dy;
          if (d2 < min_d2) { min_d2 = d2; min_id = j; }
        }
        if (min_id > furthest_reached) furthest_reached = min_id;
      }
    }

    // PathAlign's max_path_occupancy_ratio gate (depends on furthest_reached).
    bool path_align_invalid_gate_open = true;
    if (P > 1 && furthest_reached > 0) {
      unsigned int invalid_ctr = 0;
      for (unsigned int i = 0; i < furthest_reached; ++i) {
        if (!pts_valid[i]) invalid_ctr++;
      }
      const float range = std::max(static_cast<float>(furthest_reached), 1.0f);
      if (static_cast<float>(invalid_ctr) / range >
          crit_.path_align_max_path_occupancy && invalid_ctr > 2)
      {
        path_align_invalid_gate_open = false;
      }
    }

    // PathAngleCritic's posePointAngle gate (depends on furthest_reached).
    bool path_angle_gate_open = !within_goal(crit_.path_angle_threshold);
    if (path_angle_gate_open && P > 0) {
      const unsigned int offseted = std::min(
        furthest_reached + crit_.path_angle_offset_from_furthest, P - 1);
      const float dx = path.x(offseted) - robot_x;
      const float dy = path.y(offseted) - robot_y;
      const float angle_to_target = std::atan2(dy, dx);
      float ad = angle_to_target - robot_yaw;
      while (ad > M_PI)  ad -= 2.0f * M_PI;
      while (ad <= -M_PI) ad += 2.0f * M_PI;
      if (std::fabs(ad) < crit_.path_angle_max_angle_to_furthest) {
        path_angle_gate_open = false;
      }
    }

    cudaCheck(cudaMemset(d_costs_.get(), 0, B * sizeof(float)), "memset costs");

    // ── 2. 8 critics (v1: no host-side gates, weights hardcoded to yaml).
    CriticConfig ccfg_template{};
    ccfg_template.batch_size = B;
    ccfg_template.time_steps = T;
    ccfg_template.power = 1;

    // (a) ConstraintCritic — no host-side gate (always runs).
    {
      const float vx_max = settings.constraints.vx_max;
      const float vy_max = settings.constraints.vy;
      const float vx_min = settings.constraints.vx_min;
      const float max_vel = std::sqrt(vx_max * vx_max + vy_max * vy_max);
      const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
      const float min_vel = min_sgn * std::sqrt(vx_min * vx_min + vy_max * vy_max);
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.constraint_weight;
      cfg.power  = crit_.constraint_power;
      finalizeLaunchRc(
        launchConstraintCritic(cfg, d_state_vx_, holonomic ? d_state_vy_.get() : nullptr,
                               min_vel, max_vel, settings.model_dt, d_costs_),
        "constraint");
    }

    // (b) ObstaclesCritic — no gate; near_goal flag from yaml threshold.
    {
      ObstaclesConfig ocfg{};
      ocfg.batch_size               = B;
      ocfg.time_steps               = T;
      ocfg.power                    = crit_.obs_power;
      ocfg.critical_weight          = crit_.obs_critical_weight;
      ocfg.repulsion_weight         = crit_.obs_repulsion_weight;
      ocfg.collision_cost           = crit_.obs_collision_cost;
      ocfg.collision_margin_distance = crit_.obs_collision_margin;
      ocfg.inflation_radius         = crit_.obs_inflation_radius;
      ocfg.inflation_scale_factor   = crit_.obs_inflation_scale_factor;
      ocfg.circumscribed_radius     = crit_.obs_circumscribed_radius;
      ocfg.possibly_inscribed_cost  = crit_.obs_possibly_inscribed_cost;
      ocfg.tracking_unknown         = crit_.obs_tracking_unknown;
      ocfg.near_goal                = near_goal;
      ocfg.consider_footprint       = crit_.obs_consider_footprint && (fp_n_ >= 2);
      finalizeLaunchRc(
        launchObstaclesCritic(ocfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                              d_costmap_, cm_info,
                              d_fp_x_, d_fp_y_, fp_n_, d_costs_),
        "obstacles");
    }

    // (c) GoalCritic — gate: within_position_goal_tolerance(goal_threshold)
    if (P > 0 && within_goal(crit_.goal_threshold)) {
      const float gx = path.x(P - 1);
      const float gy = path.y(P - 1);
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.goal_weight;
      cfg.power  = crit_.goal_power;
      finalizeLaunchRc(
        launchGoalCritic(cfg, d_traj_x_, d_traj_y_, gx, gy, d_costs_),
        "goal");
    }

    // (d) GoalAngleCritic — same gate as GoalCritic + own threshold.
    if (P > 0 && within_goal(crit_.goal_angle_threshold)) {
      const float gyaw = path.yaws(P - 1);
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.goal_angle_weight;
      cfg.power  = crit_.goal_angle_power;
      finalizeLaunchRc(
        launchGoalAngleCritic(cfg, d_traj_yaws_, gyaw,
                              crit_.goal_angle_symmetric, d_costs_),
        "goal_angle");
    }

    // (e) PathAlignCritic — gate: !within(threshold) + path size ≥ offset +
    //                            max_path_occupancy_ratio < threshold
    if (P > 1 &&
        !within_goal(crit_.path_align_threshold) &&
        furthest_reached >= crit_.path_align_offset_from_furthest &&
        path_align_invalid_gate_open)
    {
      PathAlignConfig pcfg{};
      pcfg.batch_size            = B;
      pcfg.time_steps            = T;
      pcfg.path_segments_count   = furthest_reached;   // ← real, not P
      pcfg.trajectory_point_step = crit_.path_align_trajectory_point_step;
      pcfg.power                 = crit_.path_align_power;
      pcfg.weight                = crit_.path_align_weight;
      pcfg.use_path_orientations = crit_.path_align_use_orientations;
      finalizeLaunchRc(
        launchPathAlignCritic(pcfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                              d_path_x_, d_path_y_, d_path_yaws_,
                              d_path_int_dist_, d_path_pts_valid_, d_costs_),
        "path_align");
    }

    // (f) PathFollowCritic — gate: !within(threshold) + path ≥ 2 cells.
    // Path point used: clamp(furthest + offset, P-1), then walk forward
    // until pts_valid; matches the CPU loop.
    if (P >= 2 && !within_goal(crit_.path_follow_threshold)) {
      unsigned int idx = std::min(
        furthest_reached + crit_.path_follow_offset_from_furthest, P - 1);
      while (idx < P - 1 && !pts_valid[idx]) ++idx;
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.path_follow_weight;
      cfg.power  = crit_.path_follow_power;
      finalizeLaunchRc(
        launchPathFollowCritic(cfg, d_traj_x_, d_traj_y_,
                               path.x(idx), path.y(idx), d_costs_),
        "path_follow");
    }

    // (g) PathAngleCritic — gate: !within(threshold) + posePointAngle ≥ max.
    if (P > 0 && path_angle_gate_open) {
      const unsigned int idx = std::min(
        furthest_reached + crit_.path_angle_offset_from_furthest, P - 1);
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.path_angle_weight;
      cfg.power  = crit_.path_angle_power;
      finalizeLaunchRc(
        launchPathAngleCritic(cfg, d_traj_x_, d_traj_y_, d_traj_yaws_,
                              path.x(idx), path.y(idx), d_costs_),
        "path_angle");
    }

    // (h) PreferForwardCritic — gate: !within(threshold). Skip when robot
    // is close to goal (encourages forward motion only when traversing).
    if (!within_goal(crit_.prefer_forward_threshold)) {
      CriticConfig cfg = ccfg_template;
      cfg.weight = crit_.prefer_forward_weight;
      cfg.power  = crit_.prefer_forward_power;
      finalizeLaunchRc(
        launchPreferForwardCritic(cfg, d_state_vx_, settings.model_dt, d_costs_),
        "prefer_forward");
    }

    // ── 3. Cost-shape bias term (per dimension)
    ControlUpdateConfig ucfg{};
    ucfg.batch_size  = B;
    ucfg.time_steps  = T;
    ucfg.temperature = settings.temperature;
    ucfg.gamma       = settings.gamma;
    ucfg.std_vx      = settings.sampling_std.vx;
    ucfg.std_vy      = settings.sampling_std.vy;
    ucfg.std_wz      = settings.sampling_std.wz;
    finalizeLaunchRc(launchCostShape(ucfg, d_ctrl_vx_, d_state_cvx_, ucfg.std_vx, d_costs_), "cost_shape_vx");
    finalizeLaunchRc(launchCostShape(ucfg, d_ctrl_wz_, d_state_cwz_, ucfg.std_wz, d_costs_), "cost_shape_wz");
    if (holonomic) {
      finalizeLaunchRc(launchCostShape(ucfg, d_ctrl_vy_, d_state_cvy_, ucfg.std_vy, d_costs_), "cost_shape_vy");
    }

    // ── 4. Softmax over costs[B]
    finalizeLaunchRc(launchSoftmax(ucfg, d_costs_, d_softmax_), "softmax");

    // ── 5. Weighted average per T column → new control_sequence
    finalizeLaunchRc(launchWeightedAvg(ucfg, d_state_cvx_, d_softmax_, d_ctrl_vx_), "weighted_avg_vx");
    finalizeLaunchRc(launchWeightedAvg(ucfg, d_state_cwz_, d_softmax_, d_ctrl_wz_), "weighted_avg_wz");
    if (holonomic) {
      finalizeLaunchRc(launchWeightedAvg(ucfg, d_state_cvy_, d_softmax_, d_ctrl_vy_), "weighted_avg_vy");
    }

    // ── 6. Download control_sequence + apply constraints on host
    cudaCheck(cudaMemcpy(ctrl.vx.data(), d_ctrl_vx_.get(), T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.vx");
    cudaCheck(cudaMemcpy(ctrl.wz.data(), d_ctrl_wz_.get(), T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.wz");
    if (holonomic) {
      cudaCheck(cudaMemcpy(ctrl.vy.data(), d_ctrl_vy_.get(), T * sizeof(float), cudaMemcpyDeviceToHost), "D2H ctrl.vy");
    }
    opt.applyControlSequenceConstraintsPublic();

    (void)trajs;
    (void)costs;
  }
}

}  // namespace nav_algo_mppi_cuda
