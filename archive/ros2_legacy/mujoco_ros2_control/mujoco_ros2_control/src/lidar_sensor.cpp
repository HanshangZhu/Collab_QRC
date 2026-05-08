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

namespace mujoco_ros2_sensors {

LidarSensor::LidarSensor(rclcpp::Node::SharedPtr &node,
                           mjModel *model, mjData *data,
                           const LidarSensorConfig &cfg,
                           std::mutex *sim_step_mtx)
    : nh_(node), model_(model), data_(data),
      range_min_(cfg.range_min), range_max_(cfg.range_max),
      frame_id_(cfg.frame_id), sim_step_mtx_(sim_step_mtx)
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

    // --- Pre-compute ray directions in LiDAR-local frame ---
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
    std::vector<double> ray_dirs_world(n_rays_ * 3);
    std::vector<float> points;
    points.reserve(n_rays_ * 3);

    {
        // Lock to prevent concurrent mj_step from modifying mjData
        std::unique_lock<std::mutex> lock;
        if (sim_step_mtx_) lock = std::unique_lock<std::mutex>(*sim_step_mtx_);

        // 1. Read site position and orientation from the live sim data.
        std::memcpy(origin_copy, data_->site_xpos + site_id_ * 3, 3 * sizeof(double));
        std::memcpy(xmat_copy, data_->site_xmat + site_id_ * 9, 9 * sizeof(double));

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

        mj_multiRay(model_, data_,
                    origin_copy,              // single origin (3,)
                    ray_dirs_world.data(),    // directions (n_rays*3,)
                    nullptr,                  // geomgroup: all groups
                    1,                        // flg_static: include static geoms
                    body_id_,                 // bodyexclude: skip robot body
                    ray_geomid_.data(),       // output geom IDs
                    ray_dist_.data(),         // output distances
                    nullptr,                  // normals: not needed
                    n_rays_,
                    range_max_);              // cutoff
    }  // mutex released — remaining work is read-only on local copies

    // 4. Build hit-point list in LiDAR-local frame.
    for (int i = 0; i < n_rays_; ++i) {
        if (ray_geomid_[i] < 0) continue;
        double d = ray_dist_[i];
        if (d < range_min_ || d > range_max_) continue;

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
    msg.header.stamp    = nh_->now();
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
