// Relay per-robot namespaced TF topics to the global /tf and /tf_static.
//
// Dual-robot nav launches namespace TF via `remap /tf → /{ns}/tf`. That keeps
// per-robot listeners isolated, but tools that only subscribe to global /tf
// (notably RViz, which has no per-instance TF topic override) can't see the
// dual-robot tree.
//
// This node subscribes to a list of input namespaces and republishes every
// TFMessage on the global /tf and /tf_static. It does NOT otherwise modify
// the TFs — downstream consumers see the union of the namespaced trees.
//
// Run as:
//   ros2 run go2w_perception multi_tf_relay
//     --ros-args -p sources:="['robot_a', 'robot_b']"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using TFMessage = tf2_msgs::msg::TFMessage;

class MultiTfRelay : public rclcpp::Node {
public:
  MultiTfRelay() : rclcpp::Node("multi_tf_relay") {
    std::vector<std::string> default_sources = {"robot_a", "robot_b"};
    const auto sources = declare_parameter<std::vector<std::string>>("sources", default_sources);

    // /tf: reliable, volatile. /tf_static: reliable, transient_local.
    const rclcpp::QoS dyn_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    const rclcpp::QoS static_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();

    tf_pub_ = create_publisher<TFMessage>("/tf", dyn_qos);
    tf_static_pub_ = create_publisher<TFMessage>("/tf_static", static_qos);

    std::vector<std::string> accepted;
    accepted.reserve(sources.size());
    for (const auto & raw : sources) {
      std::string ns = raw;
      while (!ns.empty() && ns.front() == '/') ns.erase(0, 1);
      while (!ns.empty() && ns.back() == '/') ns.pop_back();
      if (ns.empty()) continue;

      subs_.push_back(create_subscription<TFMessage>(
        "/" + ns + "/tf", dyn_qos,
        [this](const TFMessage::SharedPtr msg) { tf_pub_->publish(*msg); }));
      subs_.push_back(create_subscription<TFMessage>(
        "/" + ns + "/tf_static", static_qos,
        [this](const TFMessage::SharedPtr msg) { tf_static_pub_->publish(*msg); }));
      accepted.push_back(ns);
    }

    std::string joined;
    for (size_t i = 0; i < accepted.size(); ++i) {
      if (i) joined += ", ";
      joined += accepted[i];
    }
    RCLCPP_INFO(get_logger(),
      "multi_tf_relay: relaying [%s] -> /tf and /tf_static",
      joined.c_str());
  }

private:
  rclcpp::Publisher<TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr tf_static_pub_;
  std::vector<rclcpp::Subscription<TFMessage>::SharedPtr> subs_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTfRelay>());
  rclcpp::shutdown();
  return 0;
}
