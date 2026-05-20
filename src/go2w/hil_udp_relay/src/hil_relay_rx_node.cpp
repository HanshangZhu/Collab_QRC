// hil_relay_rx_node.cpp (ROS 2) — UP-link receiver.
//
// Binds UDP ports for the messages the NX (ROS 1) sends back, reassembles +
// deserializes them, and republishes on the ROS 2 (laptop) side. Replaces the
// broken ros1_bridge 1to2 path.
//
//   UDP cmd_vel_port --> /robot/cmd_vel            (geometry_msgs/Twist)   PRIMARY
//   UDP odom_port    --> /robot/Odometry           (nav_msgs/Odometry)     viz
//   UDP trav_port    --> /robot/traversability_grid(nav_msgs/OccupancyGrid)viz
//
// Each bound port runs its own UdpReceiver on a dedicated recv thread (poll
// loop). The recv thread hands the reassembled payload to the matching publish
// helper. rclcpp publishers are thread-safe, so publishing directly from the
// recv thread is fine. See udp_protocol.hpp / udp_transport.hpp for wire format.

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "hil_udp_relay/udp_protocol.hpp"
#include "hil_udp_relay/udp_transport.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hil_udp_relay {

class HilRelayRxNode : public rclcpp::Node {
 public:
  HilRelayRxNode() : rclcpp::Node("hil_relay_rx") {
    const int cmd_vel_port = declare_parameter<int>("cmd_vel_port", 9003);
    const int odom_port = declare_parameter<int>("odom_port", 9004);
    const int trav_port = declare_parameter<int>("trav_port", 9005);
    enable_viz_ = declare_parameter<bool>("enable_viz", true);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    if (enable_viz_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/robot/Odometry", 10);
      // OccupancyGrid for RViz: map displays default to transient_local
      // (latched) + reliable. A plain volatile QoS yields the RViz warning
      // "incompatible QoS ... DURABILITY" and no map shows. Match RViz.
      trav_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/robot/traversability_grid",
          rclcpp::QoS(1).transient_local().reliable());
    }

    // cmd_vel receiver (primary).
    if (cmd_vel_rx_.open(static_cast<uint16_t>(cmd_vel_port),
                         [this](uint16_t t, const std::vector<uint8_t>& p) {
                           onPayload(t, p);
                         })) {
      spawn(cmd_vel_rx_);
    } else {
      RCLCPP_FATAL(get_logger(), "cmd_vel UDP bind failed on port %d", cmd_vel_port);
    }

    if (enable_viz_) {
      if (odom_rx_.open(static_cast<uint16_t>(odom_port),
                        [this](uint16_t t, const std::vector<uint8_t>& p) {
                          onPayload(t, p);
                        })) {
        spawn(odom_rx_);
      } else {
        RCLCPP_WARN(get_logger(), "odom UDP bind failed on port %d", odom_port);
      }
      if (trav_rx_.open(static_cast<uint16_t>(trav_port),
                        [this](uint16_t t, const std::vector<uint8_t>& p) {
                          onPayload(t, p);
                        })) {
        spawn(trav_rx_);
      } else {
        RCLCPP_WARN(get_logger(), "trav UDP bind failed on port %d", trav_port);
      }
    }

    RCLCPP_INFO(get_logger(),
                "HIL RX up: cmd_vel<-%d, odom<-%d, trav<-%d (viz=%s)",
                cmd_vel_port, odom_port, trav_port, enable_viz_ ? "on" : "off");
  }

  ~HilRelayRxNode() override {
    running_ = false;
    for (auto& t : threads_)
      if (t.joinable()) t.join();
  }

 private:
  void spawn(UdpReceiver& rx) {
    threads_.emplace_back([this, &rx]() {
      while (running_ && rclcpp::ok()) rx.poll_once(50);
    });
  }

  void onPayload(uint16_t msg_type, const std::vector<uint8_t>& payload) {
    Reader r(payload.data(), payload.size());
    switch (msg_type) {
      case MSG_TWIST: {
        Twist m;
        if (!deserialize_twist(r, m)) return;
        geometry_msgs::msg::Twist out;
        out.linear.x = m.linear[0];
        out.linear.y = m.linear[1];
        out.linear.z = m.linear[2];
        out.angular.x = m.angular[0];
        out.angular.y = m.angular[1];
        out.angular.z = m.angular[2];
        cmd_vel_pub_->publish(out);
        break;
      }
      case MSG_ODOM: {
        if (!odom_pub_) return;
        Odometry m;
        if (!deserialize_odom(r, m)) return;
        nav_msgs::msg::Odometry out;
        out.header.stamp.sec = m.header.stamp_sec;
        out.header.stamp.nanosec = m.header.stamp_nsec;
        out.header.frame_id = m.header.frame_id;
        out.child_frame_id = m.child_frame_id;
        out.pose.pose.position.x = m.position[0];
        out.pose.pose.position.y = m.position[1];
        out.pose.pose.position.z = m.position[2];
        out.pose.pose.orientation.x = m.orientation[0];
        out.pose.pose.orientation.y = m.orientation[1];
        out.pose.pose.orientation.z = m.orientation[2];
        out.pose.pose.orientation.w = m.orientation[3];
        out.twist.twist.linear.x = m.twist.linear[0];
        out.twist.twist.linear.y = m.twist.linear[1];
        out.twist.twist.linear.z = m.twist.linear[2];
        out.twist.twist.angular.x = m.twist.angular[0];
        out.twist.twist.angular.y = m.twist.angular[1];
        out.twist.twist.angular.z = m.twist.angular[2];
        odom_pub_->publish(out);
        break;
      }
      case MSG_OCCGRID: {
        if (!trav_pub_) return;
        OccupancyGrid m;
        if (!deserialize_occgrid(r, m)) return;
        nav_msgs::msg::OccupancyGrid out;
        out.header.stamp.sec = m.header.stamp_sec;
        out.header.stamp.nanosec = m.header.stamp_nsec;
        out.header.frame_id = m.header.frame_id;
        out.info.resolution = m.info.resolution;
        out.info.width = m.info.width;
        out.info.height = m.info.height;
        out.info.origin.position.x = m.info.origin_position[0];
        out.info.origin.position.y = m.info.origin_position[1];
        out.info.origin.position.z = m.info.origin_position[2];
        out.info.origin.orientation.x = m.info.origin_orientation[0];
        out.info.origin.orientation.y = m.info.origin_orientation[1];
        out.info.origin.orientation.z = m.info.origin_orientation[2];
        out.info.origin.orientation.w = m.info.origin_orientation[3];
        out.data.assign(m.data.begin(), m.data.end());
        trav_pub_->publish(out);
        break;
      }
      default:
        break;
    }
  }

  bool enable_viz_ = true;
  std::atomic<bool> running_{true};
  std::vector<std::thread> threads_;

  UdpReceiver cmd_vel_rx_;
  UdpReceiver odom_rx_;
  UdpReceiver trav_rx_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr trav_pub_;
};

}  // namespace hil_udp_relay

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hil_udp_relay::HilRelayRxNode>());
  rclcpp::shutdown();
  return 0;
}
