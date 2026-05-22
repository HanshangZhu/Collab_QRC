// hil_relay_tx_node.cpp (ROS 2) — DOWN-link transmitter.
//
// Subscribes the sensor stream on the ROS 2 (laptop) side and ships each message
// over UDP to the NX (ROS 1). Replaces the broken ros1_bridge 2to1 path.
//
//   /livox/lidar (livox_ros_driver2/CustomMsg) --> UDP nx_ip:lidar_port
//   /livox/imu   (sensor_msgs/Imu)             --> UDP nx_ip:imu_port
//   /goal_pose    (geometry_msgs/PoseStamped)   --> UDP nx_ip:goal_port
//
// Each callback converts the native ROS 2 msg to a POD struct (udp_protocol.hpp),
// serializes it, and hands it to a UdpSender which fragments + emits datagrams.
// See udp_protocol.hpp / udp_transport.hpp for the wire format and the
// little-endian / fragmentation assumptions.

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hil_udp_relay/udp_protocol.hpp"
#include "hil_udp_relay/udp_transport.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace hil_udp_relay {

class HilRelayTxNode : public rclcpp::Node {
 public:
  HilRelayTxNode() : rclcpp::Node("hil_relay_tx") {
    nx_ip_ = declare_parameter<std::string>("nx_ip", "192.168.123.18");
    const int lidar_port = declare_parameter<int>("lidar_port", 9001);
    const int imu_port = declare_parameter<int>("imu_port", 9002);
    const int goal_port = declare_parameter<int>("goal_port", 9007);

    if (!lidar_tx_.open(nx_ip_, static_cast<uint16_t>(lidar_port))) {
      RCLCPP_FATAL(get_logger(), "lidar UDP sender open failed to %s:%d",
                   nx_ip_.c_str(), lidar_port);
    }
    if (!imu_tx_.open(nx_ip_, static_cast<uint16_t>(imu_port))) {
      RCLCPP_FATAL(get_logger(), "imu UDP sender open failed to %s:%d",
                   nx_ip_.c_str(), imu_port);
    }
    if (!goal_tx_.open(nx_ip_, static_cast<uint16_t>(goal_port))) {
      RCLCPP_FATAL(get_logger(), "goal UDP sender open failed to %s:%d",
                   nx_ip_.c_str(), goal_port);
    }

    // Sensor-data QoS (best-effort) matches typical Livox/IMU publishers.
    auto qos = rclcpp::SensorDataQoS();
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", qos,
        std::bind(&HilRelayTxNode::onLidar, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", qos,
        std::bind(&HilRelayTxNode::onImu, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::QoS(10),
        std::bind(&HilRelayTxNode::onGoal, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "HIL TX up: /livox/lidar -> %s:%d, /livox/imu -> %s:%d, /goal_pose -> %s:%d",
                nx_ip_.c_str(), lidar_port, nx_ip_.c_str(), imu_port,
                nx_ip_.c_str(), goal_port);
  }

 private:
  void onLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    CustomMsg m;
    m.header.stamp_sec = msg->header.stamp.sec;
    m.header.stamp_nsec = msg->header.stamp.nanosec;
    m.header.frame_id = msg->header.frame_id;
    m.timebase = msg->timebase;
    m.point_num = msg->point_num;
    m.lidar_id = msg->lidar_id;
    m.rsvd[0] = msg->rsvd[0];
    m.rsvd[1] = msg->rsvd[1];
    m.rsvd[2] = msg->rsvd[2];
    m.points.resize(msg->points.size());
    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto& s = msg->points[i];
      auto& d = m.points[i];
      d.offset_time = s.offset_time;
      d.x = s.x;
      d.y = s.y;
      d.z = s.z;
      d.reflectivity = s.reflectivity;
      d.tag = s.tag;
      d.line = s.line;
    }
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_custommsg(w, m);
    lidar_tx_.send(MSG_LIDAR_CUSTOM, payload);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Imu m;
    m.header.stamp_sec = msg->header.stamp.sec;
    m.header.stamp_nsec = msg->header.stamp.nanosec;
    m.header.frame_id = msg->header.frame_id;
    m.orientation[0] = msg->orientation.x;
    m.orientation[1] = msg->orientation.y;
    m.orientation[2] = msg->orientation.z;
    m.orientation[3] = msg->orientation.w;
    m.angular_velocity[0] = msg->angular_velocity.x;
    m.angular_velocity[1] = msg->angular_velocity.y;
    m.angular_velocity[2] = msg->angular_velocity.z;
    m.linear_acceleration[0] = msg->linear_acceleration.x;
    m.linear_acceleration[1] = msg->linear_acceleration.y;
    m.linear_acceleration[2] = msg->linear_acceleration.z;
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_imu(w, m);
    imu_tx_.send(MSG_IMU, payload);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    PoseStamped m;
    m.header.stamp_sec = msg->header.stamp.sec;
    m.header.stamp_nsec = msg->header.stamp.nanosec;
    m.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    m.position[0] = msg->pose.position.x;
    m.position[1] = msg->pose.position.y;
    m.position[2] = msg->pose.position.z;
    m.orientation[0] = msg->pose.orientation.x;
    m.orientation[1] = msg->pose.orientation.y;
    m.orientation[2] = msg->pose.orientation.z;
    m.orientation[3] = msg->pose.orientation.w;
    std::vector<uint8_t> payload;
    Writer w(payload);
    serialize_posestamped(w, m);
    goal_tx_.send(MSG_POSESTAMPED, payload);
  }

  std::string nx_ip_;
  UdpSender lidar_tx_;
  UdpSender imu_tx_;
  UdpSender goal_tx_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace hil_udp_relay

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hil_udp_relay::HilRelayTxNode>());
  rclcpp::shutdown();
  return 0;
}
