/**
 * @file pose_sensor.cpp
 * @brief This file contains the implementation of pose sensor.
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
#include "mujoco_ros2_sensors/imu_sensor.hpp"

#include <cmath>  // std::floor
namespace mujoco_ros2_sensors {

    ImuSensor::ImuSensor(rclcpp::Node::SharedPtr &node, mjModel_ *model, mjData_ *data,
                           const ImuSensorStruct &sensor, std::atomic<bool>* stop, double frequency) {
        this->nh_ = node;
        this->mujoco_data_ = data;
        this->sensor_ = sensor;

        this->publisher_ = nh_->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
        this->imu_publisher_ = std::make_unique<ImuPublisher>(publisher_);
        imu_publisher_->lock();
        imu_publisher_->msg_.header.frame_id = sensor_.frame_id;
        imu_publisher_->unlock();


        timer_ = nh_->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&ImuSensor::update, this));
    }

    void ImuSensor::update() {
        // Dedupe: skip when sim time hasn't advanced since the last publish. The
        // wall_timer fires faster than data_->time advances, so without this we
        // emit ~47% duplicate-time samples (dt=0) -> FAST-LIO loop-back/divergence.
        const double sim_t = mujoco_data_->time;
        if (sim_t <= last_stamp_time_) {
            return;
        }
        if (imu_publisher_->trylock()) {
            last_stamp_time_ = sim_t;
            // Stamp with the true MuJoCo sim time (same as pose_sensor.cpp), NOT
            // nh_->now(). nh_->now() reads the node's /clock subscription, which
            // lags under load and jitters -> lidar/IMU stamps become inconsistent
            // with each other and with the GT/pose TF, breaking FAST-LIO's
            // lidar<->IMU time-sync and elevation_mapping's TF-at-cloud-stamp
            // lookup in the cross-host HIL. mujoco_data_->time is exact + matches
            // the published /clock.
            imu_publisher_->msg_.header.stamp.sec =
                static_cast<int32_t>(std::floor(sim_t));
            imu_publisher_->msg_.header.stamp.nanosec = static_cast<uint32_t>(
                (sim_t - std::floor(sim_t)) * 1e9);
            if (sensor_.gyro) {
                imu_publisher_->msg_.angular_velocity.x = mujoco_data_->sensordata[sensor_.gyro_sensor_adr];
                imu_publisher_->msg_.angular_velocity.y = mujoco_data_->sensordata[sensor_.gyro_sensor_adr + 1];
                imu_publisher_->msg_.angular_velocity.z = mujoco_data_->sensordata[sensor_.gyro_sensor_adr + 2];
            }

            if (sensor_.accel) {
                imu_publisher_->msg_.linear_acceleration.x = mujoco_data_->sensordata[sensor_.accel_sensor_adr];
                imu_publisher_->msg_.linear_acceleration.y = mujoco_data_->sensordata[sensor_.accel_sensor_adr + 1];
                imu_publisher_->msg_.linear_acceleration.z = mujoco_data_->sensordata[sensor_.accel_sensor_adr + 2];
            }

            imu_publisher_->unlockAndPublish();
        }

    }
}