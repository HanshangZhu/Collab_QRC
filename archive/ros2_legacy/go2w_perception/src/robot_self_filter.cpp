/*
 * robot_self_filter.cpp — drop LiDAR returns hitting OTHER robots' bodies
 * before the cloud reaches octomap.
 *
 * Problem
 * -------
 * In a multi-robot deployment, robot A's LiDAR sees robot B's body when
 * they're in line-of-sight; B's chassis returns ~30-100 hits clustered
 * in a 0.5 m radius. octomap_server has no concept of "this is another
 * agent"; it just sees those hits and marks the corresponding cells
 * OCCUPIED in A's local map. Three downstream effects all degrade
 * exploration:
 *   1. A's planner thinks there's a wall where B was, plans around it
 *      even after B has moved away.
 *   2. A's local map publishes the polluted cells to /merged_map.
 *   3. /merged_map is fed back into B's local map by map_augmenter, so
 *      B inherits A's hallucinated walls — symmetric corruption.
 *
 * Solution
 * --------
 * Each robot already publishes its own pose (in sim: /{ns}/odom/ground_truth;
 * on real swarm: whatever the comm layer broadcasts). Subscribe to peers'
 * poses, and for each incoming cloud drop points whose world (x, y) lies
 * within `peer_filter_radius_m` of any peer's last-known position.
 * Octomap then sees only the static environment.
 *
 * Pipeline:
 *   qos_bridge ──> /{ns}/registered_scan_reliable
 *                          │
 *                          ├──> Fast-LIO / pointcloud_adapter (unchanged)
 *                          │
 *                          └──> robot_self_filter ──> /{ns}/registered_scan_octomap
 *                                                             │
 *                                                             └──> octomap_server
 *
 * Fast-LIO keeps the unfiltered cloud (its ICP front-end is robust to
 * dynamic outliers and benefits from point density). Only octomap
 * consumes the filtered version, since octomap's output IS the
 * planning map.
 *
 * Why C++ rather than Python: a Mid-360 scan is ~24k points at 10 Hz
 * (240 k pts/s), with 2 peers that's 480 k distance comparisons per
 * second plus the byte-buffer rebuild for the filtered cloud. Python
 * with numpy works but adds rclpy serialisation overhead on the hot
 * path; C++ keeps the entire pipeline well under 1 ms per scan.
 */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>


using std::placeholders::_1;


class RobotSelfFilter : public rclcpp::Node {
public:
    RobotSelfFilter() : rclcpp::Node("robot_self_filter") {
        declare_parameter<std::string>("input_topic", "registered_scan_reliable");
        declare_parameter<std::string>("output_topic", "registered_scan_octomap");
        declare_parameter<std::vector<std::string>>("peer_namespaces",
                                                     std::vector<std::string>{});
        declare_parameter<std::string>("peer_pose_topic", "/odom/ground_truth");
        // Body half-bounding-circle radius. Go2W is 0.65 × 0.45 (half-
        // diag 0.40), Go2 is 0.65 × 0.30 (half-diag 0.36). 0.50 m gives
        // a safety margin without eating real environment returns near
        // the peer.
        declare_parameter<double>("peer_filter_radius_m", 0.50);
        // Drop the peer from the filter if we haven't received a pose
        // update in this long (peer offline / comms loss). LiDAR is
        // 10 Hz so missing one filter frame is far better than over-
        // trusting a stale pose.
        declare_parameter<double>("peer_pose_stale_sec", 2.0);
        // 10 s heartbeat for the operator: how many points the filter
        // is dropping per scan on average.
        declare_parameter<double>("stats_log_period_sec", 10.0);
        // Self-pose topic used to transform sensor-frame cloud points
        // into world frame before comparing to peer pose. Without this,
        // the filter only worked when cloud points coincidentally
        // happened to be world-frame (publisher-dependent), which
        // caused the asymmetric A-drops-0 / B-drops-217 bug in
        // demo3_mixed (mujoco_ros2_control's lidar_sensor.cpp emits
        // points in sensor frame, not world).
        declare_parameter<std::string>("self_odom_topic", "odom/ground_truth");

        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        peer_ns_ = get_parameter("peer_namespaces").as_string_array();
        // Strip empty entries (an empty default array shows up as [""]
        // in some launch wirings, which would build a /\<suffix\> topic).
        peer_ns_.erase(std::remove_if(peer_ns_.begin(), peer_ns_.end(),
                                       [](const std::string &s) {
                                           return s.empty();
                                       }),
                       peer_ns_.end());
        pose_topic_suffix_ = get_parameter("peer_pose_topic").as_string();
        const double radius = get_parameter("peer_filter_radius_m").as_double();
        radius_sq_ = radius * radius;
        stale_sec_ = get_parameter("peer_pose_stale_sec").as_double();
        const double stats_period = get_parameter("stats_log_period_sec").as_double();

        // /{ns}/registered_scan_reliable is published RELIABLE / VOLATILE
        // by qos_bridge in the launch — match that exactly.
        auto cloud_qos = rclcpp::QoS(5)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        // /odom/ground_truth uses sensor data QoS in the launch.
        auto odom_qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, cloud_qos,
            std::bind(&RobotSelfFilter::on_cloud, this, _1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, cloud_qos);

        // Self pose subscription — used to transform sensor-frame cloud
        // to world frame so the peer-distance check is correct
        // regardless of cloud frame_id.
        const std::string self_topic = get_parameter("self_odom_topic").as_string();
        self_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            self_topic, odom_qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                self_x_ = msg->pose.pose.position.x;
                self_y_ = msg->pose.pose.position.y;
                const auto &q = msg->pose.pose.orientation;
                // 2D yaw from quaternion (assume roll/pitch ≈ 0).
                self_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                self_pose_valid_ = true;
            });
        RCLCPP_INFO(get_logger(),
                    "  self pose: subscribed to %s (cloud points will be "
                    "rotated/translated to world before peer check)",
                    self_topic.c_str());

        for (const auto &peer : peer_ns_) {
            const std::string topic = "/" + peer + pose_topic_suffix_;
            auto sub = create_subscription<nav_msgs::msg::Odometry>(
                topic, odom_qos,
                [this, peer](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    on_peer_pose(peer, std::move(msg));
                });
            peer_subs_.push_back(sub);
            RCLCPP_INFO(get_logger(), "  peer %s: subscribed to %s",
                        peer.c_str(), topic.c_str());
        }

        if (stats_period > 0.0) {
            stats_timer_ = create_wall_timer(
                std::chrono::duration<double>(stats_period),
                std::bind(&RobotSelfFilter::stats_log, this));
        }

        RCLCPP_INFO(get_logger(),
                    "robot_self_filter started: in=%s out=%s peers=[%s] "
                    "radius=%.2fm",
                    input_topic_.c_str(), output_topic_.c_str(),
                    join(peer_ns_).c_str(), radius);
    }

private:
    struct PeerPose {
        double x;
        double y;
        rclcpp::Time stamp;
    };

    void on_peer_pose(const std::string &peer,
                      nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        peer_poses_[peer] = PeerPose{
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            this->now(),
        };
    }

    std::vector<std::pair<double, double>> active_peer_xy() const {
        std::vector<std::pair<double, double>> out;
        if (peer_poses_.empty()) return out;
        const auto now_t = this->now();
        const rclcpp::Duration stale = rclcpp::Duration::from_seconds(stale_sec_);
        out.reserve(peer_poses_.size());
        for (const auto &kv : peer_poses_) {
            const auto &p = kv.second;
            if ((now_t - p.stamp) <= stale) {
                out.emplace_back(p.x, p.y);
            }
        }
        return out;
    }

    // Find the byte offset for a named field, and verify it's float32.
    // Returns true on success, populating `*offset`.
    static bool find_xy_offset(
        const sensor_msgs::msg::PointCloud2 &msg,
        const std::string &name,
        std::size_t *offset)
    {
        for (const auto &f : msg.fields) {
            if (f.name == name) {
                if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                    return false;
                }
                *offset = static_cast<std::size_t>(f.offset);
                return true;
            }
        }
        return false;
    }

    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        const auto peers = active_peer_xy();
        if (peers.empty()) {
            // No fresh peer poses: pass through unchanged. Subscribers
            // see the same cloud they would have seen pre-filter.
            pub_->publish(*msg);
            return;
        }

        std::size_t x_off, y_off;
        if (!find_xy_offset(*msg, "x", &x_off) ||
            !find_xy_offset(*msg, "y", &y_off)) {
            pub_->publish(*msg);
            return;
        }

        const std::size_t N =
            static_cast<std::size_t>(msg->width) *
            static_cast<std::size_t>(msg->height);
        const std::size_t step = msg->point_step;
        if (N == 0 || step == 0 || msg->data.size() != N * step) {
            pub_->publish(*msg);
            return;
        }

        // Build a keep-mask in one pass over the buffer. For each point,
        // read x and y as float32 from the field offsets, transform from
        // sensor frame to world frame using cached self-pose (rotation +
        // translation), then compare against every active peer's
        // world-frame xy.
        //
        // Why the transform: mujoco_ros2_control's lidar_sensor.cpp (and
        // mujoco_lidar_node.py) emit cloud points in SENSOR frame
        // (xmat^T applied to world hit-points before publishing). The
        // peer pose comes from /odom/ground_truth, which is world.
        // Without transforming, the comparison is geometrically wrong;
        // depending on robot geometry / yaw it might coincidentally
        // drop SOME peer points (the demo3_mixed B=217/A=0 asymmetry
        // came from this).
        const bool have_self = self_pose_valid_;
        const double cos_y = have_self ? std::cos(self_yaw_) : 1.0;
        const double sin_y = have_self ? std::sin(self_yaw_) : 0.0;
        const double sx = have_self ? self_x_ : 0.0;
        const double sy = have_self ? self_y_ : 0.0;
        std::vector<uint8_t> keep(N, 1);
        std::size_t kept = 0;
        const uint8_t *base = msg->data.data();
        for (std::size_t i = 0; i < N; ++i) {
            const uint8_t *p = base + i * step;
            float x, y;
            std::memcpy(&x, p + x_off, sizeof(float));
            std::memcpy(&y, p + y_off, sizeof(float));
            // sensor-frame (x, y) → world-frame (wx, wy)
            const double wx = sx + cos_y * x - sin_y * y;
            const double wy = sy + sin_y * x + cos_y * y;
            bool drop = false;
            for (const auto &peer : peers) {
                const double dx = wx - peer.first;
                const double dy = wy - peer.second;
                if ((dx * dx + dy * dy) < radius_sq_) {
                    drop = true;
                    break;
                }
            }
            if (!drop) {
                ++kept;
            } else {
                keep[i] = 0;
            }
        }

        ++scan_count_;
        dropped_total_ += (N - kept);

        if (kept == N) {
            // Nothing to drop — avoid the buffer copy.
            pub_->publish(*msg);
            return;
        }

        // Build the filtered message. We preserve every field, just emit
        // fewer points. PointCloud2 layout: data is N rows of point_step
        // bytes; we copy the kept rows into a new contiguous buffer.
        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;
        out.height = 1;
        out.width = static_cast<uint32_t>(kept);
        out.fields = msg->fields;
        out.is_bigendian = msg->is_bigendian;
        out.point_step = msg->point_step;
        out.row_step = msg->point_step * out.width;
        out.is_dense = msg->is_dense;
        out.data.resize(static_cast<std::size_t>(out.row_step));

        uint8_t *dst = out.data.data();
        for (std::size_t i = 0; i < N; ++i) {
            if (keep[i]) {
                std::memcpy(dst, base + i * step, step);
                dst += step;
            }
        }
        pub_->publish(out);
    }

    void stats_log() {
        if (scan_count_ == 0) {
            RCLCPP_INFO(get_logger(),
                        "robot_self_filter: no scans received yet");
            return;
        }
        const double avg = static_cast<double>(dropped_total_) /
                           static_cast<double>(scan_count_);
        RCLCPP_INFO(get_logger(),
                    "robot_self_filter: %zu scans processed, %zu points "
                    "dropped total (%.1f/scan)",
                    scan_count_, dropped_total_, avg);
    }

    static std::string join(const std::vector<std::string> &v) {
        std::string out;
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (i) out += ",";
            out += v[i];
        }
        return out;
    }

    // Params (cached after construction).
    std::string input_topic_;
    std::string output_topic_;
    std::vector<std::string> peer_ns_;
    std::string pose_topic_suffix_;
    double radius_sq_;
    double stale_sec_;

    // State.
    std::unordered_map<std::string, PeerPose> peer_poses_;
    std::size_t scan_count_ = 0;
    std::size_t dropped_total_ = 0;

    // ROS.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_subs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_odom_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    // Self pose in world (map) frame for sensor→world point transform.
    double self_x_ = 0.0, self_y_ = 0.0, self_yaw_ = 0.0;
    bool self_pose_valid_ = false;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotSelfFilter>());
    rclcpp::shutdown();
    return 0;
}
