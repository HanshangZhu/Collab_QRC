/**
 * @file lidar_sensor.hpp
 * @brief LiDAR raycast sensor using mj_multiRay inside the mujoco_ros2_control plugin.
 *
 * Casts rays from a MuJoCo site using the live mjData, eliminating the sync
 * lag that occurs when raycasting in a separate process.  Publishes
 * sensor_msgs/PointCloud2 on ~/registered_scan.
 */
#ifndef MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP

#include <vector>
#include <string>
#include <cmath>
#include <mutex>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace mujoco_ros2_sensors {

/// Configuration passed from MujocoRos2Sensors to the LidarSensor constructor.
struct LidarSensorConfig {
    std::string site_name  = "livox_mid360";  ///< MuJoCo site for origin + orientation
    std::string body_name  = "base_link";     ///< Body to exclude from raycasts
    std::string frame_id   = "livox_mid360";  ///< PointCloud2 header.frame_id
    int    hz_samples      = 720;             ///< Horizontal ray count
    int    vt_samples      = 16;              ///< Vertical ray count
    double h_fov_deg       = 360.0;           ///< Horizontal FOV (degrees)
    double v_min_deg       = -7.0;            ///< Vertical FOV lower bound (degrees)
    double v_max_deg       = 52.0;            ///< Vertical FOV upper bound (degrees)
    double range_min       = 0.05;            ///< Minimum range (m)
    double range_max       = 20.0;            ///< Maximum range (m)
    double publish_rate    = 10.0;            ///< Publish rate (Hz)
};

class LidarSensor {
public:
    /**
     * @param node   ROS 2 node (owns timer, publisher, parameters)
     * @param model  Shared MuJoCo model pointer (same as physics sim)
     * @param data   Shared MuJoCo data pointer  (same as physics sim)
     * @param config LiDAR configuration
     */
    LidarSensor(rclcpp::Node::SharedPtr &node,
                mjModel *model, mjData *data,
                const LidarSensorConfig &config,
                std::mutex *sim_step_mtx = nullptr);

private:
    void update();

    rclcpp::Node::SharedPtr       nh_;
    rclcpp::TimerBase::SharedPtr  timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    mjModel *model_  = nullptr;
    mjData  *data_   = nullptr;

    // MuJoCo IDs
    int site_id_ = -1;
    int body_id_ = -1;

    // Ray configuration
    int    n_rays_;
    double range_min_;
    double range_max_;
    std::string frame_id_;

    // Pre-computed ray directions in LiDAR-local frame  (n_rays_ x 3, row-major)
    std::vector<double> ray_dirs_local_;

    // Reusable buffers for mj_multiRay output
    std::vector<double> ray_dist_;
    std::vector<int>    ray_geomid_;

    // Mutex shared with physics loop to prevent concurrent mjData access
    std::mutex *sim_step_mtx_ = nullptr;
};

}  // namespace mujoco_ros2_sensors

#endif  // MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
