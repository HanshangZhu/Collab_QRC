/**
 * @file lidar_sensor.cpp
 * @brief LiDAR raycast sensor using mj_multiRay inside the mujoco_ros2_control plugin.
 *
 * Shares mjModel/mjData with the physics sim — no separate model copy, no
 * topic-based pose sync, no timestamp mismatch.  Rays are cast against the
 * exact sim state that was most recently stepped.
 */
#include "mujoco_ros2_sensors/lidar_sensor.hpp"

#include <cstring>  // memcpy
#include <fstream>
#include <sstream>

namespace mujoco_ros2_sensors {

LidarSensor::LidarSensor(rclcpp::Node::SharedPtr &node,
                           mjModel *model, mjData *data,
                           const LidarSensorConfig &cfg,
                           std::mutex *sim_step_mtx)
    : nh_(node), model_(model), data_(data),
      range_min_(cfg.range_min), range_max_(cfg.range_max),
      frame_id_(cfg.frame_id),
      range_noise_stddev_(cfg.range_noise_stddev),
      rng_(std::random_device{}()),
      noise_dist_(0.0, cfg.range_noise_stddev > 0.0 ? cfg.range_noise_stddev : 1.0),
      sim_step_mtx_(sim_step_mtx)
{
    // --- Resolve MuJoCo IDs ---
    site_id_ = mj_name2id(model_, mjOBJ_SITE, cfg.site_name.c_str());
    if (site_id_ < 0) {
        RCLCPP_FATAL(nh_->get_logger(),
                     "LiDAR site '%s' not found in MuJoCo model", cfg.site_name.c_str());
        throw std::runtime_error("LiDAR site not found: " + cfg.site_name);
    }

    body_id_ = mj_name2id(model_, mjOBJ_BODY, cfg.body_name.c_str());
    if (body_id_ < 0) {
        RCLCPP_FATAL(nh_->get_logger(),
                     "LiDAR body '%s' not found in MuJoCo model", cfg.body_name.c_str());
        throw std::runtime_error("LiDAR body not found: " + cfg.body_name);
    }

    // --- Ray directions: CSV replay OR uniform-grid fallback ---
    //
    // CSV mode reproduces the real Risley non-repetitive scan. We slice
    // `n_rays_per_frame` directions per published frame, advancing a cursor
    // each call so coverage fills in over time. This matches what
    // elevation_mapping_cupy's visibility-cleanup + drift-compensation
    // expect from a real Livox sensor.
    //
    // CSV format (Livox-SDK/livox_laser_simulation):
    //   Time/s, Azimuth/deg, Zenith/deg
    // Zenith is measured from +Z (so horizon ≈ 90°, top of FOV ≈ 38° for Mid-360).
    // Converted to LiDAR-local Cartesian:
    //   x = sin(zenith) cos(azimuth)
    //   y = sin(zenith) sin(azimuth)
    //   z = cos(zenith)
    if (!cfg.scan_pattern_csv.empty()) {
        std::ifstream f(cfg.scan_pattern_csv);
        if (!f.is_open()) {
            RCLCPP_FATAL(nh_->get_logger(),
                         "Failed to open scan-pattern CSV '%s'", cfg.scan_pattern_csv.c_str());
            throw std::runtime_error("scan_pattern_csv not openable: " + cfg.scan_pattern_csv);
        }
        std::string line;
        std::getline(f, line);  // header
        csv_dirs_local_.reserve(800000 * 3);
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string tok;
            double az_deg = 0.0, ze_deg = 0.0;
            // col 0: time index (ignored), col 1: azimuth, col 2: zenith
            std::getline(ss, tok, ',');  // time
            if (!std::getline(ss, tok, ',')) continue;
            az_deg = std::stod(tok);
            if (!std::getline(ss, tok, ',')) continue;
            ze_deg = std::stod(tok);
            const double az = az_deg * M_PI / 180.0;
            const double ze = ze_deg * M_PI / 180.0;
            const double sin_ze = std::sin(ze);
            csv_dirs_local_.push_back(sin_ze * std::cos(az));
            csv_dirs_local_.push_back(sin_ze * std::sin(az));
            csv_dirs_local_.push_back(std::cos(ze));
        }
        csv_total_samples_ = static_cast<int>(csv_dirs_local_.size() / 3);
        if (csv_total_samples_ < cfg.n_rays_per_frame) {
            RCLCPP_FATAL(nh_->get_logger(),
                         "Scan pattern '%s' has only %d samples (< n_rays_per_frame=%d)",
                         cfg.scan_pattern_csv.c_str(), csv_total_samples_, cfg.n_rays_per_frame);
            throw std::runtime_error("scan_pattern_csv too short");
        }
        csv_mode_ = true;
        n_rays_ = cfg.n_rays_per_frame;
        ray_dirs_local_.assign(n_rays_ * 3, 0.0);  // refilled per-frame
        RCLCPP_INFO(nh_->get_logger(),
                    "LiDAR CSV scan-pattern loaded: %s (%d samples, %d rays/frame, noise σ=%.3f m)",
                    cfg.scan_pattern_csv.c_str(), csv_total_samples_, n_rays_,
                    range_noise_stddev_);
    } else {
        const double h_fov = cfg.h_fov_deg * M_PI / 180.0;
        const double v_min = cfg.v_min_deg * M_PI / 180.0;
        const double v_max = cfg.v_max_deg * M_PI / 180.0;

        n_rays_ = cfg.hz_samples * cfg.vt_samples;
        ray_dirs_local_.resize(n_rays_ * 3);

        int idx = 0;
        for (int h = 0; h < cfg.hz_samples; ++h) {
            double h_angle = -h_fov / 2.0 + h_fov * h / cfg.hz_samples;
            for (int v = 0; v < cfg.vt_samples; ++v) {
                double v_angle = v_min + (v_max - v_min) * v / (cfg.vt_samples - 1);
                double cos_v = std::cos(v_angle);
                ray_dirs_local_[idx * 3 + 0] = cos_v * std::cos(h_angle);
                ray_dirs_local_[idx * 3 + 1] = cos_v * std::sin(h_angle);
                ray_dirs_local_[idx * 3 + 2] = std::sin(v_angle);
                ++idx;
            }
        }
    }

    // --- Allocate output buffers ---
    ray_dist_.resize(n_rays_);
    ray_geomid_.resize(n_rays_);

    // --- ROS publisher (BEST_EFFORT to match downstream QoS) ---
    rclcpp::QoS qos(5);
    qos.best_effort();
    pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", qos);

    // --- Timer ---
    timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(1.0 / cfg.publish_rate),
        std::bind(&LidarSensor::update, this));

    RCLCPP_INFO(nh_->get_logger(),
                "LiDAR sensor started: site=%s, body=%s, %dx%d=%d rays, %.0f Hz, "
                "range [%.2f, %.1f]m, frame=%s",
                cfg.site_name.c_str(), cfg.body_name.c_str(),
                cfg.hz_samples, cfg.vt_samples, n_rays_,
                cfg.publish_rate, cfg.range_min, cfg.range_max,
                cfg.frame_id.c_str());
}

// ---------------------------------------------------------------------------
void LidarSensor::update()
{
    // All mjData reads and mj_multiRay must be protected from concurrent mj_step.
    double origin_copy[3];
    double xmat_copy[9];
    double sim_time_capture = 0.0;
    std::vector<double> ray_dirs_world(n_rays_ * 3);
    std::vector<float> points;
    points.reserve(n_rays_ * 3);

    // 0. CSV mode: refill ray_dirs_local_ with the next `n_rays_` slice of the
    //    scan-pattern table, wrap-around at end. Done outside the sim-step
    //    mutex (pure local arithmetic on csv_dirs_local_).
    if (csv_mode_) {
        for (int i = 0; i < n_rays_; ++i) {
            const int src = (csv_offset_ + i) % csv_total_samples_;
            ray_dirs_local_[i * 3 + 0] = csv_dirs_local_[src * 3 + 0];
            ray_dirs_local_[i * 3 + 1] = csv_dirs_local_[src * 3 + 1];
            ray_dirs_local_[i * 3 + 2] = csv_dirs_local_[src * 3 + 2];
        }
        csv_offset_ = (csv_offset_ + n_rays_) % csv_total_samples_;
    }

    {
        // Lock to prevent concurrent mj_step from modifying mjData
        std::unique_lock<std::mutex> lock;
        if (sim_step_mtx_) lock = std::unique_lock<std::mutex>(*sim_step_mtx_);

        // 1. Read site position and orientation from the live sim data.
        std::memcpy(origin_copy, data_->site_xpos + site_id_ * 3, 3 * sizeof(double));
        std::memcpy(xmat_copy, data_->site_xmat + site_id_ * 9, 9 * sizeof(double));
        // Capture the exact MuJoCo sim time of THIS raycast (under the same lock)
        // for the cloud stamp below — see the header.stamp note at publish.
        sim_time_capture = data_->time;

        // 2. Rotate ray directions from local to world frame.
        for (int i = 0; i < n_rays_; ++i) {
            const double lx = ray_dirs_local_[i * 3 + 0];
            const double ly = ray_dirs_local_[i * 3 + 1];
            const double lz = ray_dirs_local_[i * 3 + 2];
            ray_dirs_world[i * 3 + 0] = xmat_copy[0] * lx + xmat_copy[1] * ly + xmat_copy[2] * lz;
            ray_dirs_world[i * 3 + 1] = xmat_copy[3] * lx + xmat_copy[4] * ly + xmat_copy[5] * lz;
            ray_dirs_world[i * 3 + 2] = xmat_copy[6] * lx + xmat_copy[7] * ly + xmat_copy[8] * lz;
        }

        // 3. Cast all rays in one C call.
        std::fill(ray_dist_.begin(), ray_dist_.end(), range_max_);
        std::fill(ray_geomid_.begin(), ray_geomid_.end(), -1);

        // MuJoCo 3.2 (mjVERSION_HEADER 320) ADDED the `normals` output
        // argument to mj_multiRay (previous comment had the direction
        // inverted; the version check originally read `< 320` and broke
        // on MuJoCo 3.6.0 with "too few arguments"). Pass nullptr on
        // 3.2+; omit on older headers that don't have the slot.
        mj_multiRay(model_, data_,
                    origin_copy,              // single origin (3,)
                    ray_dirs_world.data(),    // directions (n_rays*3,)
                    nullptr,                  // geomgroup: all groups
                    1,                        // flg_static: include static geoms
                    body_id_,                 // bodyexclude: skip robot body
                    ray_geomid_.data(),       // output geom IDs
                    ray_dist_.data(),         // output distances
#if mjVERSION_HEADER >= 320 && mjVERSION_HEADER < 330
                    nullptr,                  // normals: added in 3.2, removed in 3.3
#endif
                    n_rays_,
                    range_max_);              // cutoff
    }  // mutex released — remaining work is read-only on local copies

    // 4. Build hit-point list in LiDAR-local frame.
    for (int i = 0; i < n_rays_; ++i) {
        if (ray_geomid_[i] < 0) continue;
        double d = ray_dist_[i];
        if (d < range_min_ || d > range_max_) continue;

        // Range noise: Gaussian σ (m). Models Livox Mid-360 ≤2 cm @ 10 m
        // precision spec. Applied AFTER range gating so very close hits
        // can't be pushed below range_min_ by noise alone — keep semantics
        // identical between noise-on and noise-off in the rejection step.
        if (range_noise_stddev_ > 0.0) {
            d += noise_dist_(rng_);
            if (d < range_min_) d = range_min_;
        }

        double wx = ray_dirs_world[i * 3 + 0] * d;
        double wy = ray_dirs_world[i * 3 + 1] * d;
        double wz = ray_dirs_world[i * 3 + 2] * d;

        float lx = static_cast<float>(xmat_copy[0] * wx + xmat_copy[3] * wy + xmat_copy[6] * wz);
        float ly = static_cast<float>(xmat_copy[1] * wx + xmat_copy[4] * wy + xmat_copy[7] * wz);
        float lz = static_cast<float>(xmat_copy[2] * wx + xmat_copy[5] * wy + xmat_copy[8] * wz);

        points.push_back(lx);
        points.push_back(ly);
        points.push_back(lz);
    }

    int n_valid = static_cast<int>(points.size() / 3);
    if (n_valid == 0) return;

    // 5. Build PointCloud2 message.
    sensor_msgs::msg::PointCloud2 msg;
    // Stamp with the true MuJoCo sim time of the raycast (captured under the
    // sim-step lock above), NOT nh_->now(). nh_->now() reads the node's /clock
    // subscription, which lags + jitters under HIL load -> the cloud stamp drifts
    // 1-2 s behind the actual /clock and is inconsistent with the IMU and the
    // GT/pose TF. That desyncs FAST-LIO's lidar<->IMU alignment and makes
    // elevation_mapping's TF-at-cloud-stamp lookup throw ExtrapolationException
    // (-> trav_grid never builds). mujoco_data_->time is exact and matches the
    // published /clock, so all sim sensors share one consistent time base.
    msg.header.stamp.sec     = static_cast<int32_t>(std::floor(sim_time_capture));
    msg.header.stamp.nanosec = static_cast<uint32_t>(
        (sim_time_capture - std::floor(sim_time_capture)) * 1e9);
    msg.header.frame_id = frame_id_;
    msg.height          = 1;
    msg.width           = n_valid;
    msg.is_bigendian    = false;
    msg.point_step      = 12;  // 3 x float32
    msg.row_step        = msg.point_step * msg.width;
    msg.is_dense        = true;

    msg.fields.resize(3);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[0].count = 1;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[1].count = 1;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[2].count = 1;

    msg.data.resize(n_valid * 12);
    std::memcpy(msg.data.data(), points.data(), n_valid * 12);

    pub_->publish(msg);
}

}  // namespace mujoco_ros2_sensors
