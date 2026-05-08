/**
 * @file mujoco_ros2_sensors.cpp
 * @brief This file contains the implementation of Sensor handler.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cstdlib>
#include <string>
#include <utility>
#include "mujoco_ros2_sensors/mujoco_ros2_sensors.hpp"

namespace {
    // SIM-ONLY env-var override. Controls how many rays mj_multiRay() casts
    // per sensor update inside the MuJoCo raycast plugin. Has NO effect on
    // real hardware (real MID-360's point rate is fixed by its firmware).
    // Export before launching to change sim ray count without rebuilding:
    //   MUJOCO_LIDAR_HZ_SAMPLES  (horizontal sim rays)
    //   MUJOCO_LIDAR_VT_SAMPLES  (vertical sim rays)
    // Total sim rays = hz * vt. Defaults: 1000 * 20 = 20000, chosen to
    // emulate the real MID-360's 200k pts/s at 10 Hz. Lower to speed up
    // MuJoCo's mj_ray work (biggest sim RTF lever); fidelity of the
    // simulated cloud drops proportionally.
    int env_int(const char *name, int fallback) {
        const char *v = std::getenv(name);
        if (!v || !*v) return fallback;
        try { return std::max(1, std::stoi(v)); }
        catch (...) { return fallback; }
    }
}

namespace mujoco_ros2_sensors {
    MujocoRos2Sensors::MujocoRos2Sensors(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, mjModel_ *model,
                                         mjData_ *data, std::map<std::string, Sensors> sensors, const std::string& ns, std::mutex* sim_step_mtx) {
        this->executor_ = executor;
        this->mujoco_model_ = model;
        this->mujoco_data_ = data;
        this->sensors_ = std::move(sensors);
        this->ns_ = ns;
        this->sim_step_mtx_ = sim_step_mtx;

        std::vector<PoseSensorStruct> pose_sensors;
        std::vector<WrenchSensorStruct> wrench_sensors;
        std::vector<ImuSensorStruct> imu_sensors;
        for (const auto &sensor : sensors_) {
            PoseSensorStruct pose_sensor;
            WrenchSensorStruct wrench_sensor;
            ImuSensorStruct imu_sensor;
            for (int i = 0; i < sensor.second.sensor_ids.size(); i++) {
                auto &sensor_id = sensor.second.sensor_ids[i];
                auto &sensor_type = sensor.second.sensor_types[i];
                auto &sensor_name = sensor.second.sensor_names[i];
                auto &sensor_address = sensor.second.sensor_addresses[i];
                auto &dimension = sensor.second.sensor_dimensions[i];
                if (sensor_type == mjSENS_FRAMEPOS) {
                    if (pose_sensor.position) {
                        RCLCPP_WARN(rclcpp::get_logger("pose_sensor_registration"), "Position sensor with address %d already registered, ignoring second sensor with address %d", pose_sensor.position_sensor_adr, sensor_address);
                        continue;
                    }
                    pose_sensor.body_name = sensor.first;
                    pose_sensor.position_sensor_adr = sensor_address;
                    pose_sensor.position = true;
                } else if (sensor_type == mjSENS_FRAMEQUAT) {
                    if (pose_sensor.orientation) {
                        RCLCPP_WARN(rclcpp::get_logger("pose_sensor_registration"), "Orientation sensor with address %d already registered, ignoring second sensor with address %d", pose_sensor.orientation_sensor_adr, sensor_address);
                        continue;
                    }
                    pose_sensor.body_name = sensor.first;
                    pose_sensor.orientation_sensor_adr = sensor_address;
                    pose_sensor.orientation = true;
                } else if (sensor.second.sensor_types[i] == mjSENS_FORCE) {
                    if (wrench_sensor.force) {
                        RCLCPP_WARN(rclcpp::get_logger("wrench_sensor_registration"), "Force sensor with address %d already registered, ignoring second sensor with address %d", wrench_sensor.force_sensor_adr, sensor_address);
                        continue;
                    }
                    wrench_sensor.body_name = sensor.first;
                    wrench_sensor.force_sensor_adr = sensor_address;
                    wrench_sensor.force = true;
                } else if (sensor.second.sensor_types[i] == mjSENS_TORQUE) {
                    if (wrench_sensor.torque) {
                        RCLCPP_WARN(rclcpp::get_logger("wrench_sensor_registration"), "Torque sensor with address %d already registered, ignoring second sensor with address %d", wrench_sensor.torque_sensor_adr, sensor_address);
                        continue;
                    }
                    wrench_sensor.body_name = sensor.first;
                    wrench_sensor.torque_sensor_adr = sensor_address;
                    wrench_sensor.torque = true;
                } else  if (sensor.second.sensor_types[i] == mjSENS_ACCELEROMETER) {
                    if (imu_sensor.accel) {
                        RCLCPP_WARN(rclcpp::get_logger("imu_sensor_registration"), "Accel sensor with address %d already registered, ignoring second sensor with address %d", imu_sensor.accel_sensor_adr, sensor_address);
                        continue;
                    }
                    imu_sensor.body_name = sensor.first;
                    imu_sensor.accel_sensor_adr = sensor_address;
                    imu_sensor.accel = true;
                } else if (sensor.second.sensor_types[i] == mjSENS_GYRO) {
                    if (imu_sensor.gyro) {
                        RCLCPP_WARN(rclcpp::get_logger("imu_sensor_registration"), "Gyro sensor with address %d already registered, ignoring second sensor with address %d", imu_sensor.gyro_sensor_adr, sensor_address);
                        continue;
                    }
                    imu_sensor.body_name = sensor.first;
                    imu_sensor.gyro_sensor_adr = sensor_address;
                    imu_sensor.gyro = true;
                }
                if ((sensor_type == mjSENS_FRAMEPOS || sensor_type == mjSENS_FRAMEQUAT) && pose_sensor.frame_id.empty()) {
                    pose_sensor.frame_id = get_frame_id(sensor_id);
                } else if ((sensor_type == mjSENS_FORCE || sensor_type == mjSENS_TORQUE) && wrench_sensor.frame_id.empty()) {
                    wrench_sensor.frame_id = get_frame_id(sensor_id);
                } else if ((sensor_type == mjSENS_ACCELEROMETER || sensor_type == mjSENS_GYRO) && imu_sensor.frame_id.empty()) {
                    imu_sensor.frame_id = get_frame_id(sensor_id);
                }
            }
            if (!pose_sensor.frame_id.empty()) {
                pose_sensors.push_back(pose_sensor);
            }
            if (!wrench_sensor.frame_id.empty()) {
                wrench_sensors.push_back(wrench_sensor);
            }
            if (!imu_sensor.frame_id.empty()) {
                imu_sensors.push_back(imu_sensor);
            }
        }

        register_pose_sensors(pose_sensors);
        register_wrench_sensors(wrench_sensors);
        register_imu_sensors(imu_sensors);
        register_lidar_sensors();
    }

    void MujocoRos2Sensors::register_lidar_sensors() {
        // Auto-discover lidar sites: look for known site names.
        // For dual-robot MJCF, also look for b_-prefixed variants.
        // Order of preference per robot: unitree_l1, livox_mid360.
        // Real hardware specs — see Livox MID-360 datasheet.
        // range_min=0.1 matches the real sensor (point cloud starts at
        // 0.1 m; accuracy not guaranteed below 0.2 m). Do NOT lower to
        // 0.05 — tuning obstacle avoidance to rely on sub-0.1 m sensing
        // breaks on real hardware.
        // rays: 20000 pts/frame = 200k pts/s ÷ 10 Hz (real MID-360).
        //   Distributed as ~1000 h × 20 v for uniform grid approximation.
        //   Real sensor uses non-repetitive Risley prism pattern, not a
        //   grid — uniform is adequate for Cartographer / terrain_analysis
        //   which treat the cloud as an unstructured set.
        // range_max=40 matches 10%-reflectivity spec (70 m at 80%).
        // TODO: add Gaussian noise (σ=0.02 m) to each mj_ray return to
        //   match the ≤2 cm @ 10 m range precision spec. Currently rays
        //   are perfect, giving Cartographer unrealistically clean scans.
        //   Needs a C++ change in lidar_sensor.cpp after mj_ray() call.
        // Sim-only ray-count overrides (see helper at top of file). These
        // set how many rays MuJoCo's mj_multiRay() fires per frame; the
        // real sensor is not affected.
        const int mid360_hz = env_int("MUJOCO_LIDAR_HZ_SAMPLES", 1000);
        const int mid360_vt = env_int("MUJOCO_LIDAR_VT_SAMPLES", 20);
        const std::vector<std::pair<std::string, LidarSensorConfig>> known_lidars = {
            {"unitree_l1", {"unitree_l1", "base_link", "livox_mid360",
                            360, 60, 360.0, 0.0, 90.0, 0.1, 20.0, 11.0}},
            {"livox_mid360", {"livox_mid360", "base_link", "livox_mid360",
                              mid360_hz, mid360_vt, 360.0, -7.0, 52.0, 0.1, 40.0, 10.0}},
        };

        // Prefixes to try: "" for Robot A, "b_" for Robot B.
        // Extend this list for more robots if needed.
        static const std::vector<std::string> prefixes = {"", "b_"};

        int registered = 0;
        for (const auto &prefix : prefixes) {
            bool found_for_prefix = false;
            for (const auto &[base_site_name, default_cfg] : known_lidars) {
                std::string site_name = prefix + base_site_name;
                int sid = mj_name2id(mujoco_model_, mjOBJ_SITE, site_name.c_str());
                if (sid < 0) continue;

                LidarSensorConfig cfg = default_cfg;
                cfg.site_name = site_name;
                cfg.frame_id = prefix + default_cfg.frame_id;
                cfg.body_name = prefix + default_cfg.body_name;

                // Unique node name: "mujoco_lidar_sensor" for Robot A,
                // "b_mujoco_lidar_sensor" for Robot B, etc.
                std::string node_name = prefix.empty()
                    ? "mujoco_lidar_sensor"
                    : prefix.substr(0, prefix.size() - 1) + "_mujoco_lidar_sensor";

                // Remap registered_scan → {node_name}/registered_scan so each
                // LiDAR gets a unique topic (critical for dual-robot setups).
                rclcpp::NodeOptions lidar_opts;
                lidar_opts.parameter_overrides({{"use_sim_time", true}});
                lidar_opts.arguments({
                    "--ros-args", "-r",
                    "registered_scan:=" + node_name + "/registered_scan"
                });
                auto node = rclcpp::Node::make_shared(node_name, ns_, lidar_opts);
                executor_->add_node(node);

                lidar_sensor_nodes_.push_back(node);
                lidar_sensor_objs_.push_back(std::make_shared<LidarSensor>(
                    node, mujoco_model_, mujoco_data_, cfg, sim_step_mtx_));

                RCLCPP_INFO(rclcpp::get_logger("lidar_sensor_registration"),
                            "LiDAR sensor registered: site=%s, frame=%s, node=%s",
                            cfg.site_name.c_str(), cfg.frame_id.c_str(), node_name.c_str());
                ++registered;
                found_for_prefix = true;
                break;  // one lidar per prefix (prefer unitree_l1 over livox_mid360)
            }
        }

        if (registered == 0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_sensor_registration"),
                        "No LiDAR site found (tried: unitree_l1, livox_mid360, b_unitree_l1, b_livox_mid360)");
        }
    }

    MujocoRos2Sensors::~MujocoRos2Sensors() {
        for (auto &node : pose_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : pose_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : wrench_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : wrench_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : imu_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : imu_sensor_objs_) {
            obj.reset();
        }
        for (auto &obj : lidar_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : lidar_sensor_nodes_) {
            node.reset();
        }
    }

    std::string MujocoRos2Sensors::get_frame_id(int sensor_id) {
        if (mujoco_model_->sensor_refid[sensor_id] == -1) {
            const auto &obj_type = mujoco_model_->sensor_objtype[sensor_id];
            const auto &obj_id = mujoco_model_->sensor_objid[sensor_id];
            return mj_id2name(mujoco_model_, obj_type, obj_id);
        } else {
            const auto &frame_id = mujoco_model_->sensor_refid[sensor_id];
            const auto &frame_type = mujoco_model_->sensor_reftype[sensor_id];
            return mj_id2name(mujoco_model_, frame_type, frame_id);
        }
    }

    void MujocoRos2Sensors::register_pose_sensors(const std::vector<PoseSensorStruct> &sensors) {
        pose_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.position) {
                    value = "Position";
                } else if (sensor.orientation) {
                    value = "Orientation";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("pose_sensor_registration"), "Pose sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = pose_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_pose_sensor", ns_, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            pose_sensor_objs_.at(i).reset(new PoseSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
            RCLCPP_INFO(rclcpp::get_logger("pose_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }

    void MujocoRos2Sensors::register_wrench_sensors(const std::vector<WrenchSensorStruct> &sensors) {
        wrench_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.force) {
                    value = "Force";
                } else if (sensor.torque) {
                    value = "Torque";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("wrench_sensor_registration"), "Wrench sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = wrench_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_wrench_sensor", ns_, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            wrench_sensor_objs_.at(i).reset(new WrenchSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
            RCLCPP_INFO(rclcpp::get_logger("wrench_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }

    void MujocoRos2Sensors::register_imu_sensors(const std::vector<ImuSensorStruct> &sensors) {
        imu_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.gyro) {
                    value = "Gyro";
                } else if (sensor.accel) {
                    value = "Accel";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("imu_sensor_registration"), "IMU sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = imu_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_imu_sensor", ns_, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            imu_sensor_objs_.at(i).reset(new ImuSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 1000.0));
            RCLCPP_INFO(rclcpp::get_logger("imu_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }
}
