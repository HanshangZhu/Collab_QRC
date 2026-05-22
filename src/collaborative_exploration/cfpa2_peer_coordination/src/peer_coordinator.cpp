// peer_coordinator.cpp — decentralised peer coordinator implementation.

#include "cfpa2_peer_coordination/peer_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/point.hpp"

namespace cfpa2_peer_coordination {

namespace {

constexpr const char * kCfpa2FrontierMarkerNs = "cfpa2_frontiers";

// Strip a single leading slash from a namespace string. ROS namespaces
// may come in either with or without the leading "/"; downstream topic
// construction assumes no leading slash.
std::string strip_leading_slash(std::string s)
{
  if (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  return s;
}

// QoS profile for peer-state heartbeats: best-effort, latest-only.
// Heartbeats are intentionally lossy; a missed message means we use
// the previous state (and eventually trigger a freshness timeout).
// Reliable delivery would queue stale heartbeats behind retried ones,
// which is the wrong behaviour for state broadcasts.
rclcpp::QoS peer_state_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

// QoS profile for negotiation request/response messages: reliable.
// Point-to-point protocol messages where dropped delivery causes
// correctness issues — the requester would hang waiting for a response
// that never arrives. Reliable + KEEP_LAST gives at-least-once
// delivery with bounded queueing.
rclcpp::QoS negotiation_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  // Volatile (not transient_local) because late-joining subscribers
  // don't need to receive old (now-stale) requests.
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

}  // namespace

// Constructor + parameter setup

PeerCoordinatorNode::PeerCoordinatorNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("cfpa2_peer_coordinator", node_options)
{
  declare_parameters();
  read_parameters();
  build_topic_names();
  wire_pub_sub();
  start_timers();
  log_startup_banner();
}

void PeerCoordinatorNode::declare_parameters()
{
  declare_parameter<std::string>("robot_id", "robot_a");
  declare_parameter<std::string>("robot_namespace", "robot_a");
  declare_parameter<std::vector<std::string>>("peer_namespaces", {"robot_b"});

  declare_parameter<double>("peer_timeout_sec", 10.0);
  declare_parameter<double>("claim_timeout_sec", 30.0);
  declare_parameter<double>("peer_state_rate_hz", 2.0);
  declare_parameter<double>("negotiation_rate_hz", 1.0);
  declare_parameter<double>("negotiation_cooldown_sec", 2.0);

  declare_parameter<double>("request_timeout_sec", 2.0);
  declare_parameter<double>("request_reject_backoff_sec", 2.0);
  declare_parameter<double>("request_timeout_backoff_sec", 1.0);
  declare_parameter<bool>("enable_negotiation_requests", true);

  declare_parameter<std::string>("odom_topic_suffix", "/odom/nav");
  declare_parameter<std::string>("frontier_markers_topic", "/mtare/frontier_markers");
  declare_parameter<double>("blocked_frontiers_rate_hz", 1.0);

  declare_parameter<double>("mdvrp_time_limit_sec", 0.5);
  declare_parameter<int>("mdvrp_span_cost_coefficient", 100);
}

void PeerCoordinatorNode::read_parameters()
{
  robot_id_ = get_parameter("robot_id").as_string();
  robot_namespace_ = strip_leading_slash(get_parameter("robot_namespace").as_string());
  peer_namespaces_ = get_parameter("peer_namespaces").as_string_array();
  peer_ids_ = peer_namespaces_;  // assumption: peer_id == peer_namespace

  peer_timeout_sec_ = get_parameter("peer_timeout_sec").as_double();
  claim_timeout_sec_ = get_parameter("claim_timeout_sec").as_double();
  peer_state_rate_hz_ = get_parameter("peer_state_rate_hz").as_double();
  negotiation_rate_hz_ = get_parameter("negotiation_rate_hz").as_double();
  negotiation_cooldown_sec_ = get_parameter("negotiation_cooldown_sec").as_double();

  request_timeout_sec_ = get_parameter("request_timeout_sec").as_double();
  request_reject_backoff_sec_ = get_parameter("request_reject_backoff_sec").as_double();
  request_timeout_backoff_sec_ = get_parameter("request_timeout_backoff_sec").as_double();
  enable_negotiation_requests_ = get_parameter("enable_negotiation_requests").as_bool();

  odom_topic_suffix_ = get_parameter("odom_topic_suffix").as_string();
  frontier_markers_topic_ = get_parameter("frontier_markers_topic").as_string();
  blocked_frontiers_rate_hz_ = get_parameter("blocked_frontiers_rate_hz").as_double();

  mdvrp_time_limit_sec_ = get_parameter("mdvrp_time_limit_sec").as_double();
  mdvrp_span_cost_coefficient_ =
      static_cast<int>(get_parameter("mdvrp_span_cost_coefficient").as_int());

  for (const auto & peer_id : peer_ids_) {
    peer_info_.emplace(peer_id, PeerInfo{});
    requester_states_.emplace(peer_id, RequesterState{});
    peer_claims_.emplace(peer_id, std::vector<ClaimedFrontier>{});
  }
}

void PeerCoordinatorNode::build_topic_names()
{
  const std::string ns = "/" + robot_namespace_;
  own_peer_state_topic_ = ns + "/cfpa2_peer_coordination/peer_state";
  own_blocked_frontiers_topic_ = ns + "/cfpa2_peer_coordination/blocked_frontiers";
  own_request_inbox_topic_ = ns + "/cfpa2_peer_coordination/inbox/negotiation_request";
  own_response_inbox_topic_ = ns + "/cfpa2_peer_coordination/inbox/negotiation_response";
  own_odom_topic_ = ns + odom_topic_suffix_;

  for (std::size_t i = 0; i < peer_ids_.size(); ++i) {
    const std::string & peer_id = peer_ids_[i];
    const std::string peer_ns = "/" + strip_leading_slash(peer_namespaces_[i]);
    peer_state_topics_[peer_id] =
        peer_ns + "/cfpa2_peer_coordination/peer_state";
    peer_request_inbox_topics_[peer_id] =
        peer_ns + "/cfpa2_peer_coordination/inbox/negotiation_request";
    peer_response_inbox_topics_[peer_id] =
        peer_ns + "/cfpa2_peer_coordination/inbox/negotiation_response";
  }
}

void PeerCoordinatorNode::wire_pub_sub()
{
  const rclcpp::QoS ps_qos = peer_state_qos();
  const rclcpp::QoS neg_qos = negotiation_qos();

  // Own odom subscriber.
  subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      own_odom_topic_, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odom(msg);
      }));
  RCLCPP_INFO(get_logger(), "Subscribed to own odometry on %s",
      own_odom_topic_.c_str());

  // Local frontier-markers subscriber.
  subs_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
      frontier_markers_topic_, 10,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        on_frontier_markers(msg);
      }));
  RCLCPP_INFO(get_logger(), "Subscribed to frontier markers on %s",
      frontier_markers_topic_.c_str());

  // Own PeerState publisher.
  peer_state_pub_ = create_publisher<PeerState>(own_peer_state_topic_, ps_qos);

  // Blocked frontiers publisher.
  blocked_frontiers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      own_blocked_frontiers_topic_, 1);

  // Per-peer negotiation request/response publishers + peer_state subs.
  for (std::size_t i = 0; i < peer_ids_.size(); ++i) {
    const std::string & peer_id = peer_ids_[i];

    peer_request_pubs_[peer_id] = create_publisher<NegotiationRequest>(
        peer_request_inbox_topics_.at(peer_id), neg_qos);
    peer_response_pubs_[peer_id] = create_publisher<NegotiationResponse>(
        peer_response_inbox_topics_.at(peer_id), neg_qos);

    RCLCPP_INFO(get_logger(),
        "Created negotiation publishers for peer %s: request -> %s, response -> %s",
        peer_id.c_str(),
        peer_request_inbox_topics_.at(peer_id).c_str(),
        peer_response_inbox_topics_.at(peer_id).c_str());

    // peer_id captured by value so each subscription remembers its peer.
    const std::string peer_state_topic = peer_state_topics_.at(peer_id);
    subs_.push_back(create_subscription<PeerState>(
        peer_state_topic, ps_qos,
        [this, peer_id](const PeerState::SharedPtr msg) {
          on_peer_state(msg, peer_id);
        }));
  }

  // Own negotiation inbox subs.
  subs_.push_back(create_subscription<NegotiationRequest>(
      own_request_inbox_topic_, neg_qos,
      [this](const NegotiationRequest::SharedPtr msg) {
        on_negotiation_request(msg);
      }));
  subs_.push_back(create_subscription<NegotiationResponse>(
      own_response_inbox_topic_, neg_qos,
      [this](const NegotiationResponse::SharedPtr msg) {
        on_negotiation_response(msg);
      }));
  RCLCPP_INFO(get_logger(),
      "Subscribed to own negotiation inbox: request <- %s, response <- %s",
      own_request_inbox_topic_.c_str(),
      own_response_inbox_topic_.c_str());
}

void PeerCoordinatorNode::start_timers()
{
  const auto from_hz = [](double hz) {
    const double period_sec = 1.0 / std::max(hz, 1e-6);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_sec));
  };

  peer_state_timer_ = create_wall_timer(
      from_hz(peer_state_rate_hz_),
      [this]() { publish_peer_state(); });
  negotiation_timer_ = create_wall_timer(
      from_hz(negotiation_rate_hz_),
      [this]() { decide_negotiation(); });
  blocked_frontiers_timer_ = create_wall_timer(
      from_hz(blocked_frontiers_rate_hz_),
      [this]() { publish_blocked_frontiers(); });
}

void PeerCoordinatorNode::log_startup_banner()
{
  RCLCPP_INFO(get_logger(),
      "PeerCoordinatorNode starting | robot_id=%s namespace=%s "
      "protocol_version=%u",
      robot_id_.c_str(), robot_namespace_.c_str(),
      static_cast<unsigned>(kProtocolVersion));

  std::string peers_csv;
  for (std::size_t i = 0; i < peer_namespaces_.size(); ++i) {
    if (i > 0) peers_csv += ", ";
    peers_csv += peer_namespaces_[i];
  }
  RCLCPP_INFO(get_logger(), "Peer namespaces: [%s]", peers_csv.c_str());

  RCLCPP_INFO(get_logger(),
      "Publishing own PeerState on %s", own_peer_state_topic_.c_str());
  RCLCPP_INFO(get_logger(),
      "Publishing blocked frontiers on %s at %.2f Hz",
      own_blocked_frontiers_topic_.c_str(), blocked_frontiers_rate_hz_);
  RCLCPP_INFO(get_logger(),
      "MDVRP solver | time_limit_sec=%.2fs span_cost=%d",
      mdvrp_time_limit_sec_, mdvrp_span_cost_coefficient_);
  RCLCPP_INFO(get_logger(),
      "Negotiation requester | enabled=%s request_timeout_sec=%.2f "
      "cooldown_sec=%.2f",
      enable_negotiation_requests_ ? "true" : "false",
      request_timeout_sec_, negotiation_cooldown_sec_);
}

// Helpers

std_msgs::msg::Header PeerCoordinatorNode::make_header()
{
  std_msgs::msg::Header h;
  h.stamp = ns_to_time_msg(now_ns());
  h.frame_id = "map";
  return h;
}

builtin_interfaces::msg::Time PeerCoordinatorNode::ns_to_time_msg(std::uint64_t ns) const
{
  builtin_interfaces::msg::Time t;
  if (ns == 0) {
    t.sec = 0;
    t.nanosec = 0;
    return t;
  }
  t.sec = static_cast<std::int32_t>(ns / 1'000'000'000ULL);
  t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000ULL);
  return t;
}

std::string PeerCoordinatorNode::next_request_id()
{
  ++request_counter_;
  return robot_id_ + "-" + std::to_string(request_counter_);
}

std::uint64_t PeerCoordinatorNode::now_ns()
{
  return static_cast<std::uint64_t>(this->get_clock()->now().nanoseconds());
}

// Subscription callbacks

void PeerCoordinatorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_pose_ = msg->pose.pose;
}

void PeerCoordinatorNode::on_frontier_markers(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::vector<Point3> frontiers;
  frontiers.reserve(msg->markers.size());

  for (const auto & marker : msg->markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD) continue;
    if (marker.ns != kCfpa2FrontierMarkerNs) continue;
    frontiers.push_back({
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z,
    });
  }

  std::sort(frontiers.begin(), frontiers.end());  // lexicographic on (x, y, z)
  local_frontiers_ = std::move(frontiers);

  RCLCPP_DEBUG(get_logger(),
      "Stored %zu local frontier candidates", local_frontiers_.size());
}

void PeerCoordinatorNode::on_peer_state(
    const PeerState::SharedPtr msg, const std::string & peer_id)
{
  auto info_it = peer_info_.find(peer_id);
  if (info_it == peer_info_.end()) {
    RCLCPP_WARN(get_logger(),
        "Received PeerState from unknown peer_id=%s; ignoring",
        peer_id.c_str());
    return;
  }

  if (msg->robot_id != peer_id) {
    RCLCPP_WARN(get_logger(),
        "Ignoring PeerState for expected peer_id=%s: message robot_id=%s",
        peer_id.c_str(), msg->robot_id.c_str());
    return;
  }

  if (msg->protocol_version != kProtocolVersion) {
    RCLCPP_WARN(get_logger(),
        "Protocol version mismatch from peer_id=%s: theirs=%u, ours=%u. Ignoring",
        peer_id.c_str(),
        static_cast<unsigned>(msg->protocol_version),
        static_cast<unsigned>(kProtocolVersion));
    return;
  }

  info_it->second.last_state = *msg;
  info_it->second.last_received_ns = now_ns();
  info_it->second.was_fresh = true;

  peer_claims_[peer_id] = msg->claimed_frontiers;

  expire_stale_claims();
  resolve_own_claim_conflicts();

  RCLCPP_DEBUG(get_logger(),
      "Received PeerState from %s at ns=%llu; stored %zu peer claims",
      peer_id.c_str(),
      static_cast<unsigned long long>(info_it->second.last_received_ns),
      peer_claims_[peer_id].size());
}

void PeerCoordinatorNode::on_negotiation_request(
    const NegotiationRequest::SharedPtr msg)
{
  if (msg->protocol_version != kProtocolVersion) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationRequest with protocol mismatch: "
        "request_id=%s; theirs=%u; ours=%u",
        msg->request_id.c_str(),
        static_cast<unsigned>(msg->protocol_version),
        static_cast<unsigned>(kProtocolVersion));
    return;
  }

  if (msg->responder_id != robot_id_) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationRequest not addressed to this robot: "
        "request_id=%s; requester_id=%s; responder_id=%s; this_robot=%s",
        msg->request_id.c_str(), msg->requester_id.c_str(),
        msg->responder_id.c_str(), robot_id_.c_str());
    return;
  }

  const bool requester_known = std::find(
      peer_ids_.begin(), peer_ids_.end(), msg->requester_id) != peer_ids_.end();
  if (!requester_known) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationRequest from unknown requester: "
        "request_id=%s; requester_id=%s",
        msg->request_id.c_str(), msg->requester_id.c_str());
    return;
  }

  const auto [accepted, reason] = validate_negotiation_request(*msg);

  NegotiationResponse response;
  response.header = make_header();
  response.request_id = msg->request_id;
  response.requester_id = msg->requester_id;
  response.responder_id = robot_id_;
  response.response_stamp = ns_to_time_msg(now_ns());
  response.accepted = accepted;
  response.reason = reason;
  response.responder_last_interaction_attempt_stamp =
      ns_to_time_msg(last_interaction_attempt_ns_);
  response.protocol_version = kProtocolVersion;

  if (accepted) {
    response.accepted_requester_claims = msg->requester_claims;
    response.accepted_responder_claims = msg->responder_claims;
  }

  // Publish BEFORE local commit — two-generals catch: send accept
  // before mutating local state so the responder is not left in a
  // committed state if the accept is dropped on the wire.
  peer_response_pubs_.at(msg->requester_id)->publish(response);

  if (accepted) {
    // Chunk D: cache peer claims immediately so the requester-side
    // suppression check sees consistent state before the next PeerState
    // heartbeat arrives.
    own_claims_ = msg->responder_claims;
    peer_claims_[msg->requester_id] = msg->requester_claims;
    last_successful_interaction_ns_ = now_ns();

    RCLCPP_INFO(get_logger(),
        "NegotiationRequest accepted | request_id=%s; from=%s; "
        "requester_claims=%zu; responder_claims=%zu; reason='%s'",
        msg->request_id.c_str(), msg->requester_id.c_str(),
        msg->requester_claims.size(), msg->responder_claims.size(),
        reason.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
        "NegotiationRequest rejected | request_id=%s; from=%s; "
        "requester_claims=%zu; responder_claims=%zu; reason='%s'",
        msg->request_id.c_str(), msg->requester_id.c_str(),
        msg->requester_claims.size(), msg->responder_claims.size(),
        reason.c_str());
  }

  RCLCPP_INFO(get_logger(),
      "NegotiationResponse sent | request_id=%s; to=%s; accepted=%s; reason='%s'",
      msg->request_id.c_str(), msg->requester_id.c_str(),
      accepted ? "true" : "false", reason.c_str());
}

void PeerCoordinatorNode::on_negotiation_response(
    const NegotiationResponse::SharedPtr msg)
{
  if (msg->protocol_version != kProtocolVersion) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse with protocol mismatch: "
        "request_id=%s; theirs=%u; ours=%u",
        msg->request_id.c_str(),
        static_cast<unsigned>(msg->protocol_version),
        static_cast<unsigned>(kProtocolVersion));
    return;
  }

  if (msg->requester_id != robot_id_) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse not addressed to this requester: "
        "requester_id=%s; responder_id=%s; this_robot=%s",
        msg->requester_id.c_str(), msg->responder_id.c_str(),
        robot_id_.c_str());
    return;
  }

  const bool responder_known = std::find(
      peer_ids_.begin(), peer_ids_.end(), msg->responder_id) != peer_ids_.end();
  if (!responder_known) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse from unknown responder: "
        "request_id=%s; responder_id=%s",
        msg->request_id.c_str(), msg->responder_id.c_str());
    return;
  }

  auto state_it = requester_states_.find(msg->responder_id);
  if (state_it == requester_states_.end()) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse with no requester state: "
        "request_id=%s; responder_id=%s",
        msg->request_id.c_str(), msg->responder_id.c_str());
    return;
  }
  RequesterState & state = state_it->second;

  if (state.state != RequesterStateKind::Requesting) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse while not REQUESTING: "
        "request_id=%s; responder_id=%s; state=%s",
        msg->request_id.c_str(), msg->responder_id.c_str(),
        state.state == RequesterStateKind::Idle ? "IDLE" : "REQUESTING");
    return;
  }

  if (!state.in_flight_request_id ||
      *state.in_flight_request_id != msg->request_id) {
    RCLCPP_WARN(get_logger(),
        "Ignoring NegotiationResponse for stale/unknown request_id: "
        "received=%s; expected=%s; responder_id=%s",
        msg->request_id.c_str(),
        state.in_flight_request_id ? state.in_flight_request_id->c_str() : "(none)",
        msg->responder_id.c_str());
    return;
  }

  const std::uint64_t now = now_ns();

  if (msg->accepted) {
    const bool accepted_matches_proposal =
        claims_match_within_tolerance(
            msg->accepted_requester_claims, state.proposed_own_claims) &&
        claims_match_within_tolerance(
            msg->accepted_responder_claims, state.proposed_peer_claims);

    if (!accepted_matches_proposal) {
      state.consecutive_rejects += 1;
      RCLCPP_WARN(get_logger(),
          "Ignoring accepted NegotiationResponse because accepted claims "
          "do not match in-flight proposal | request_id=%s; peer_id=%s; "
          "consecutive_rejects=%d",
          msg->request_id.c_str(), msg->responder_id.c_str(),
          state.consecutive_rejects);

      state.state = RequesterStateKind::Idle;
      state.in_flight_request_id.reset();
      state.request_sent_ns = 0;
      state.proposed_own_claims.clear();
      state.proposed_peer_claims.clear();
      state.backoff_until_ns =
          now + static_cast<std::uint64_t>(request_reject_backoff_sec_ * 1e9);
      return;
    }

    // Chunk D: cache peer claims immediately so future suppression
    // checks have consistent state before the next PeerState heartbeat.
    own_claims_ = msg->accepted_requester_claims;
    peer_claims_[msg->responder_id] = msg->accepted_responder_claims;

    state.state = RequesterStateKind::Idle;
    state.in_flight_request_id.reset();
    state.request_sent_ns = 0;
    state.proposed_own_claims.clear();
    state.proposed_peer_claims.clear();
    state.consecutive_rejects = 0;
    state.backoff_until_ns =
        now + static_cast<std::uint64_t>(negotiation_cooldown_sec_ * 1e9);

    last_successful_interaction_ns_ = now;

    RCLCPP_INFO(get_logger(),
        "Negotiation accepted; committed own claims | request_id=%s; "
        "peer_id=%s; own_claims=%zu",
        msg->request_id.c_str(), msg->responder_id.c_str(),
        own_claims_.size());
  } else {
    state.consecutive_rejects += 1;

    // Exponential backoff on reject, capped at 4x.
    const int exp_shift = std::max(0, state.consecutive_rejects - 1);
    const int multiplier_raw = (exp_shift >= 30) ? 4 : (1 << exp_shift);
    const int multiplier = std::min(4, multiplier_raw);
    const double backoff_sec =
        request_reject_backoff_sec_ * static_cast<double>(multiplier);

    state.state = RequesterStateKind::Idle;
    state.in_flight_request_id.reset();
    state.request_sent_ns = 0;
    state.proposed_own_claims.clear();
    state.proposed_peer_claims.clear();
    state.backoff_until_ns = now + static_cast<std::uint64_t>(backoff_sec * 1e9);

    RCLCPP_WARN(get_logger(),
        "Negotiation rejected | request_id=%s; peer_id=%s; "
        "reason='%s'; consecutive_rejects=%d; backoff_sec=%.2f",
        msg->request_id.c_str(), msg->responder_id.c_str(),
        msg->reason.c_str(), state.consecutive_rejects, backoff_sec);
  }
}

// Timer ticks

void PeerCoordinatorNode::publish_peer_state()
{
  PeerState msg;
  msg.header = make_header();
  msg.robot_id = robot_id_;

  if (latest_pose_) {
    msg.pose = *latest_pose_;
  } else {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Publishing heartbeat with no odom yet on %s",
        own_peer_state_topic_.c_str());
  }

  expire_stale_claims();
  msg.claimed_frontiers = own_claims_;

  msg.last_interaction_attempt_stamp = ns_to_time_msg(last_interaction_attempt_ns_);
  msg.last_successful_interaction_stamp =
      ns_to_time_msg(last_successful_interaction_ns_);
  msg.protocol_version = kProtocolVersion;

  peer_state_pub_->publish(msg);
}

void PeerCoordinatorNode::publish_blocked_frontiers()
{
  expire_stale_claims();

  geometry_msgs::msg::PoseArray msg;
  msg.header = make_header();

  for (const auto & [peer_id, claims] : peer_claims_) {
    (void)peer_id;
    for (const auto & claim : claims) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = claim.position.x;
      pose.position.y = claim.position.y;
      pose.position.z = claim.position.z;
      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
  }

  blocked_frontiers_pub_->publish(msg);

  RCLCPP_DEBUG(get_logger(),
      "Published %zu blocked frontier(s)", msg.poses.size());
}

void PeerCoordinatorNode::decide_negotiation()
{
  const std::vector<std::string> fresh = fresh_peer_ids();
  const std::vector<std::string> stale = stale_peer_ids();
  const std::vector<Point3> available = available_local_frontiers();

  if (enable_negotiation_requests_) {
    // Negotiation must solve over the FULL local frontier set. Peer
    // claims are used by the local planner as avoidance constraints,
    // not as inputs that remove candidates from the negotiation problem.
    tick_requester_state(fresh, local_frontiers_);
  }

  if (!logged_negotiation_stub_) {
    std::string fresh_csv, stale_csv;
    for (std::size_t i = 0; i < fresh.size(); ++i) {
      if (i > 0) fresh_csv += ", ";
      fresh_csv += fresh[i];
    }
    for (std::size_t i = 0; i < stale.size(); ++i) {
      if (i > 0) stale_csv += ", ";
      stale_csv += stale[i];
    }
    RCLCPP_INFO(get_logger(),
        "decide_negotiation stub reached; fresh_peers=[%s]; stale_peers=[%s]; "
        "local_frontiers=%zu; available_frontiers=%zu; "
        "negotiation_requests_enabled=%s",
        fresh_csv.c_str(), stale_csv.c_str(),
        local_frontiers_.size(), available.size(),
        enable_negotiation_requests_ ? "true" : "false");
    logged_negotiation_stub_ = true;
  }
}

// Freshness + staleness handling

bool PeerCoordinatorNode::peer_is_fresh(const std::string & peer_id)
{
  auto it = peer_info_.find(peer_id);
  if (it == peer_info_.end() || !it->second.last_state.has_value()) {
    return false;
  }
  const std::uint64_t now = now_ns();
  const std::uint64_t last = it->second.last_received_ns;
  if (last > now) return true;  // future timestamp — treat as fresh
  const double age_sec = static_cast<double>(now - last) / 1e9;
  return age_sec <= peer_timeout_sec_;
}

std::vector<std::string> PeerCoordinatorNode::fresh_peer_ids()
{
  std::vector<std::string> out;
  for (const auto & peer_id : peer_ids_) {
    if (peer_is_fresh(peer_id)) out.push_back(peer_id);
  }
  return out;
}

std::vector<std::string> PeerCoordinatorNode::stale_peer_ids()
{
  std::vector<std::string> out;
  for (const auto & peer_id : peer_ids_) {
    if (!peer_is_fresh(peer_id)) out.push_back(peer_id);
  }
  return out;
}

void PeerCoordinatorNode::handle_peer_staleness()
{
  const std::uint64_t now = now_ns();

  for (const auto & peer_id : peer_ids_) {
    auto info_it = peer_info_.find(peer_id);
    const bool is_fresh = peer_is_fresh(peer_id);

    if (is_fresh) {
      if (info_it != peer_info_.end()) {
        info_it->second.was_fresh = true;
      }
      continue;
    }

    if (info_it == peer_info_.end() || !info_it->second.was_fresh) {
      continue;  // already stale; log only on transition
    }

    const std::size_t dropped_claims = peer_claims_[peer_id].size();
    peer_claims_[peer_id].clear();

    auto rs_it = requester_states_.find(peer_id);
    std::optional<std::string> in_flight_id;
    bool was_in_flight = false;
    if (rs_it != requester_states_.end()) {
      was_in_flight = rs_it->second.state == RequesterStateKind::Requesting;
      in_flight_id = rs_it->second.in_flight_request_id;
      rs_it->second = RequesterState{};
    }
    if (was_in_flight) {
      RCLCPP_WARN(get_logger(),
          "Peer became stale while negotiation request was in-flight; "
          "cancelling requester state | peer_id=%s; request_id=%s",
          peer_id.c_str(),
          in_flight_id ? in_flight_id->c_str() : "(none)");
    }

    info_it->second.was_fresh = false;

    const double age_sec = info_it->second.last_received_ns > 0
        ? static_cast<double>(now - info_it->second.last_received_ns) / 1e9
        : std::numeric_limits<double>::infinity();

    RCLCPP_WARN(get_logger(),
        "Peer became stale; dropping peer claims | "
        "peer_id=%s; age_sec=%.2f; peer_timeout_sec=%.2f; dropped_claims=%zu",
        peer_id.c_str(), age_sec, peer_timeout_sec_, dropped_claims);
  }
}

// Claim management + conflict resolution

std::uint64_t PeerCoordinatorNode::claim_stamp_ns(const ClaimedFrontier & claim) const
{
  return static_cast<std::uint64_t>(claim.claim_stamp.sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(claim.claim_stamp.nanosec);
}

bool PeerCoordinatorNode::claim_is_fresh(const ClaimedFrontier & claim)
{
  const std::uint64_t cns = claim_stamp_ns(claim);
  const std::uint64_t now = now_ns();
  if (cns > now) return true;  // future stamp — treat as fresh
  const double age_sec = static_cast<double>(now - cns) / 1e9;
  return age_sec <= claim_timeout_sec_;
}

void PeerCoordinatorNode::expire_stale_claims()
{
  // Direct build-new-vector port of the Python list-comprehension.
  std::vector<ClaimedFrontier> own_kept;
  own_kept.reserve(own_claims_.size());
  for (const auto & c : own_claims_) {
    if (claim_is_fresh(c)) own_kept.push_back(c);
  }
  own_claims_ = std::move(own_kept);

  for (auto & [peer_id, claims] : peer_claims_) {
    (void)peer_id;
    std::vector<ClaimedFrontier> kept;
    kept.reserve(claims.size());
    for (const auto & c : claims) {
      if (claim_is_fresh(c)) kept.push_back(c);
    }
    claims = std::move(kept);
  }
}

bool PeerCoordinatorNode::same_frontier_position(
    const Point3 & a, const Point3 & b) const
{
  return distance_xy(a, b) <= kFrontierMatchToleranceM;
}

bool PeerCoordinatorNode::claim_wins_against(
    const ClaimedFrontier & a, const ClaimedFrontier & b) const
{
  const std::uint64_t a_ns = claim_stamp_ns(a);
  const std::uint64_t b_ns = claim_stamp_ns(b);
  if (a_ns != b_ns) return a_ns < b_ns;          // earlier timestamp wins
  return a.claimed_by < b.claimed_by;             // lex tie-break
}

void PeerCoordinatorNode::resolve_own_claim_conflicts()
{
  std::vector<ClaimedFrontier> surviving;
  surviving.reserve(own_claims_.size());
  std::vector<std::string> dropped_details;

  for (const auto & own_claim : own_claims_) {
    const Point3 own_point = point_msg_to_tuple(own_claim.position);
    const ClaimedFrontier * winning_peer_claim = nullptr;

    for (const auto & [peer_id, peer_claim_list] : peer_claims_) {
      (void)peer_id;
      for (const auto & peer_claim : peer_claim_list) {
        const Point3 peer_point = point_msg_to_tuple(peer_claim.position);
        if (!same_frontier_position(own_point, peer_point)) continue;
        if (claim_wins_against(peer_claim, own_claim)) {
          winning_peer_claim = &peer_claim;
          break;
        }
      }
      if (winning_peer_claim != nullptr) break;
    }

    if (winning_peer_claim == nullptr) {
      surviving.push_back(own_claim);
    } else {
      std::ostringstream oss;
      oss << "own_claim=(" << own_point[0] << ", " << own_point[1] << ") "
          << "lost_to=" << winning_peer_claim->claimed_by << " "
          << "peer_claim_stamp=" << claim_stamp_ns(*winning_peer_claim) << " "
          << "own_claim_stamp=" << claim_stamp_ns(own_claim);
      dropped_details.push_back(oss.str());
    }
  }

  const std::size_t dropped = own_claims_.size() - surviving.size();
  own_claims_ = std::move(surviving);

  if (dropped > 0) {
    std::string details_csv;
    for (std::size_t i = 0; i < dropped_details.size(); ++i) {
      if (i > 0) details_csv += "; ";
      details_csv += dropped_details[i];
    }
    RCLCPP_WARN(get_logger(),
        "Dropped own claim(s) due to deterministic peer conflict resolution | "
        "dropped=%zu; details=[%s]",
        dropped, details_csv.c_str());
  }
}

bool PeerCoordinatorNode::claims_match_within_tolerance(
    const std::vector<ClaimedFrontier> & a,
    const std::vector<ClaimedFrontier> & b) const
{
  if (a.size() != b.size()) return false;

  std::vector<bool> matched(b.size(), false);

  for (const auto & claim_a : a) {
    const Point3 point_a = point_msg_to_tuple(claim_a.position);
    bool found = false;
    for (std::size_t idx = 0; idx < b.size(); ++idx) {
      if (matched[idx]) continue;
      const Point3 point_b = point_msg_to_tuple(b[idx].position);
      if (same_frontier_position(point_a, point_b)) {
        matched[idx] = true;
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

bool PeerCoordinatorNode::claim_position_known_locally(
    const ClaimedFrontier & claim) const
{
  const Point3 claim_point = point_msg_to_tuple(claim.position);
  for (const auto & frontier : local_frontiers_) {
    if (same_frontier_position(claim_point, frontier)) return true;
  }
  return false;
}

bool PeerCoordinatorNode::frontier_blocked_by_peer_claim(
    const Point3 & frontier) const
{
  for (const auto & [peer_id, claims] : peer_claims_) {
    (void)peer_id;
    for (const auto & claim : claims) {
      const Point3 claim_point = point_msg_to_tuple(claim.position);
      if (same_frontier_position(frontier, claim_point)) return true;
    }
  }
  return false;
}

std::vector<Point3> PeerCoordinatorNode::available_local_frontiers()
{
  handle_peer_staleness();
  expire_stale_claims();

  std::vector<Point3> out;
  out.reserve(local_frontiers_.size());
  for (const auto & frontier : local_frontiers_) {
    if (!frontier_blocked_by_peer_claim(frontier)) out.push_back(frontier);
  }
  return out;
}

ClaimedFrontier PeerCoordinatorNode::point3_to_claim_for(
    const Point3 & point,
    const std::string & claimed_by,
    double information_gain)
{
  ClaimedFrontier claim;
  claim.position.x = point[0];
  claim.position.y = point[1];
  claim.position.z = point[2];
  claim.claimed_by = claimed_by;
  claim.claim_stamp = ns_to_time_msg(now_ns());
  claim.information_gain = information_gain;
  return claim;
}

// Negotiation logic

std::optional<std::pair<std::vector<ClaimedFrontier>, std::vector<ClaimedFrontier>>>
PeerCoordinatorNode::build_negotiation_proposal(
    const std::string & peer_id,
    const std::vector<Point3> & candidate_frontiers)
{
  if (!latest_pose_) {
    RCLCPP_DEBUG(get_logger(),
        "Skipping negotiation proposal for %s: no local odom yet",
        peer_id.c_str());
    return std::nullopt;
  }
  if (!peer_is_fresh(peer_id)) return std::nullopt;
  if (candidate_frontiers.empty()) return std::nullopt;

  auto info_it = peer_info_.find(peer_id);
  if (info_it == peer_info_.end() || !info_it->second.last_state.has_value()) {
    return std::nullopt;
  }

  std::unordered_map<std::string, Point3> robot_poses;
  robot_poses[robot_id_] = pose_msg_to_tuple(*latest_pose_);
  robot_poses[peer_id] = pose_msg_to_tuple(info_it->second.last_state->pose);

  const Assignment assignment = solve_frontier_assignment(
      robot_poses, candidate_frontiers,
      /*min_robot_to_frontier_dist=*/0.25,
      mdvrp_time_limit_sec_,
      mdvrp_span_cost_coefficient_);

  std::vector<Point3> own_points;
  std::vector<Point3> peer_points;
  if (auto it = assignment.find(robot_id_); it != assignment.end()) {
    own_points = it->second;
  }
  if (auto it = assignment.find(peer_id); it != assignment.end()) {
    peer_points = it->second;
  }

  std::vector<ClaimedFrontier> proposed_own_claims;
  proposed_own_claims.reserve(own_points.size());
  for (const auto & p : own_points) {
    proposed_own_claims.push_back(point3_to_claim_for(p, robot_id_));
  }

  std::vector<ClaimedFrontier> proposed_peer_claims;
  proposed_peer_claims.reserve(peer_points.size());
  for (const auto & p : peer_points) {
    proposed_peer_claims.push_back(point3_to_claim_for(p, peer_id));
  }

  if (proposed_own_claims.empty() && proposed_peer_claims.empty()) {
    return std::nullopt;
  }

  return std::make_pair(
      std::move(proposed_own_claims), std::move(proposed_peer_claims));
}

std::optional<std::pair<std::vector<ClaimedFrontier>, std::vector<ClaimedFrontier>>>
PeerCoordinatorNode::responder_local_resolve(const NegotiationRequest & msg)
{
  if (!latest_pose_) {
    RCLCPP_DEBUG(get_logger(),
        "Responder validation cannot re-solve: no local odom yet");
    return std::nullopt;
  }
  if (!peer_is_fresh(msg.requester_id)) {
    RCLCPP_DEBUG(get_logger(),
        "Responder validation cannot re-solve: requester heartbeat stale | "
        "requester_id=%s", msg.requester_id.c_str());
    return std::nullopt;
  }
  if (local_frontiers_.empty()) {
    RCLCPP_DEBUG(get_logger(),
        "Responder validation cannot re-solve: no local frontiers");
    return std::nullopt;
  }

  auto info_it = peer_info_.find(msg.requester_id);
  if (info_it == peer_info_.end() || !info_it->second.last_state.has_value()) {
    RCLCPP_DEBUG(get_logger(),
        "Responder validation cannot re-solve: no requester PeerState | "
        "requester_id=%s", msg.requester_id.c_str());
    return std::nullopt;
  }

  std::unordered_map<std::string, Point3> robot_poses;
  robot_poses[robot_id_] = pose_msg_to_tuple(*latest_pose_);
  robot_poses[msg.requester_id] =
      pose_msg_to_tuple(info_it->second.last_state->pose);

  const Assignment assignment = solve_frontier_assignment(
      robot_poses, local_frontiers_,
      /*min_robot_to_frontier_dist=*/0.25,
      mdvrp_time_limit_sec_,
      mdvrp_span_cost_coefficient_);

  std::vector<Point3> requester_points;
  std::vector<Point3> responder_points;
  if (auto it = assignment.find(msg.requester_id); it != assignment.end()) {
    requester_points = it->second;
  }
  if (auto it = assignment.find(robot_id_); it != assignment.end()) {
    responder_points = it->second;
  }

  std::vector<ClaimedFrontier> expected_requester_claims;
  expected_requester_claims.reserve(requester_points.size());
  for (const auto & p : requester_points) {
    expected_requester_claims.push_back(
        point3_to_claim_for(p, msg.requester_id));
  }

  std::vector<ClaimedFrontier> expected_responder_claims;
  expected_responder_claims.reserve(responder_points.size());
  for (const auto & p : responder_points) {
    expected_responder_claims.push_back(point3_to_claim_for(p, robot_id_));
  }

  if (expected_requester_claims.empty() && expected_responder_claims.empty()) {
    return std::nullopt;
  }

  return std::make_pair(
      std::move(expected_requester_claims),
      std::move(expected_responder_claims));
}

std::pair<bool, std::string> PeerCoordinatorNode::validate_negotiation_request(
    const NegotiationRequest & msg)
{
  // 1. Crossing resolution: if we're already REQUESTING toward the
  //    same peer, the lex-smaller robot_id keeps its in-flight request
  //    and rejects the incoming. The loser cancels its request and
  //    evaluates the incoming on its merits.
  auto own_rs_it = requester_states_.find(msg.requester_id);
  if (own_rs_it != requester_states_.end() &&
      own_rs_it->second.state == RequesterStateKind::Requesting) {
    if (robot_id_ < msg.requester_id) {
      RCLCPP_INFO(get_logger(),
          "Crossing negotiation detected; keeping own request and rejecting "
          "incoming | incoming_request_id=%s; own_request_id=%s; "
          "this_robot=%s; requester=%s",
          msg.request_id.c_str(),
          own_rs_it->second.in_flight_request_id
              ? own_rs_it->second.in_flight_request_id->c_str() : "(none)",
          robot_id_.c_str(), msg.requester_id.c_str());
      return {false, reject_reason::kCrossingLost};
    }
    // We lose the crossing: cancel own in-flight request, evaluate
    // incoming.
    const std::optional<std::string> cancelled_id =
        own_rs_it->second.in_flight_request_id;
    own_rs_it->second = RequesterState{};
    RCLCPP_INFO(get_logger(),
        "Crossing negotiation detected; cancelling own request and "
        "evaluating incoming | cancelled_request_id=%s; "
        "incoming_request_id=%s; this_robot=%s; requester=%s",
        cancelled_id ? cancelled_id->c_str() : "(none)",
        msg.request_id.c_str(),
        robot_id_.c_str(), msg.requester_id.c_str());
  }

  // 2. claimed_by sanity.
  for (const auto & claim : msg.requester_claims) {
    if (claim.claimed_by != msg.requester_id) {
      RCLCPP_WARN(get_logger(),
          "NegotiationRequest has requester claim assigned to wrong robot | "
          "request_id=%s; claim.claimed_by=%s; expected=%s",
          msg.request_id.c_str(), claim.claimed_by.c_str(),
          msg.requester_id.c_str());
      return {false, reject_reason::kProtocolError};
    }
  }
  for (const auto & claim : msg.responder_claims) {
    if (claim.claimed_by != robot_id_) {
      RCLCPP_WARN(get_logger(),
          "NegotiationRequest has responder claim assigned to wrong robot | "
          "request_id=%s; claim.claimed_by=%s; expected=%s",
          msg.request_id.c_str(), claim.claimed_by.c_str(),
          robot_id_.c_str());
      return {false, reject_reason::kProtocolError};
    }
  }

  // 3. frontier_unknown_locally — every proposed claim must match a
  //    local frontier within FRONTIER_MATCH_TOLERANCE. Cheap fast-fail
  //    before re-running MDVRP.
  for (const auto & claim : msg.requester_claims) {
    if (!claim_position_known_locally(claim)) {
      const Point3 cp = point_msg_to_tuple(claim.position);
      RCLCPP_WARN(get_logger(),
          "NegotiationRequest references frontier unknown locally | "
          "request_id=%s; claim=(%.2f, %.2f); claimed_by=%s",
          msg.request_id.c_str(), cp[0], cp[1], claim.claimed_by.c_str());
      return {false, reject_reason::kFrontierUnknownLocally};
    }
  }
  for (const auto & claim : msg.responder_claims) {
    if (!claim_position_known_locally(claim)) {
      const Point3 cp = point_msg_to_tuple(claim.position);
      RCLCPP_WARN(get_logger(),
          "NegotiationRequest references frontier unknown locally | "
          "request_id=%s; claim=(%.2f, %.2f); claimed_by=%s",
          msg.request_id.c_str(), cp[0], cp[1], claim.claimed_by.c_str());
      return {false, reject_reason::kFrontierUnknownLocally};
    }
  }

  // 4. claim_overlap — proposed responder claims must not partially
  //    overlap existing own claims. Re-committing exactly the same
  //    responder claim set IS allowed.
  if (!claims_match_within_tolerance(msg.responder_claims, own_claims_)) {
    for (const auto & proposed : msg.responder_claims) {
      const Point3 proposed_point = point_msg_to_tuple(proposed.position);
      for (const auto & existing : own_claims_) {
        const Point3 existing_point = point_msg_to_tuple(existing.position);
        if (same_frontier_position(proposed_point, existing_point)) {
          RCLCPP_WARN(get_logger(),
              "NegotiationRequest overlaps existing responder claim | "
              "request_id=%s; overlap=(%.2f, %.2f)",
              msg.request_id.c_str(),
              proposed_point[0], proposed_point[1]);
          return {false, reject_reason::kClaimOverlap};
        }
      }
    }
  }

  // 5. local_solve_mismatch — re-run MDVRP from the responder's view
  //    and compare via tolerance-based set matching.
  const auto local_resolve = responder_local_resolve(msg);
  if (!local_resolve) {
    return {false, reject_reason::kNoLocalExpectedProposal};
  }
  const auto & [expected_requester, expected_responder] = *local_resolve;

  const bool requester_matches =
      claims_match_within_tolerance(msg.requester_claims, expected_requester);
  const bool responder_matches =
      claims_match_within_tolerance(msg.responder_claims, expected_responder);

  if (!requester_matches || !responder_matches) {
    RCLCPP_WARN(get_logger(),
        "NegotiationRequest rejected: local MDVRP solve mismatch | "
        "request_id=%s; requester_claims=%zu expected_requester_claims=%zu; "
        "responder_claims=%zu expected_responder_claims=%zu",
        msg.request_id.c_str(),
        msg.requester_claims.size(), expected_requester.size(),
        msg.responder_claims.size(), expected_responder.size());
    return {false, reject_reason::kLocalSolveMismatch};
  }

  return {true, reject_reason::kAccepted};
}

void PeerCoordinatorNode::tick_requester_state(
    const std::vector<std::string> & fresh_peers,
    const std::vector<Point3> & candidate_frontiers)
{
  if (!enable_negotiation_requests_) return;

  const std::uint64_t now = now_ns();
  const std::uint64_t request_timeout_ns =
      static_cast<std::uint64_t>(request_timeout_sec_ * 1e9);

  for (const auto & peer_id : fresh_peers) {
    auto it = requester_states_.find(peer_id);
    if (it == requester_states_.end()) continue;
    RequesterState & state = it->second;

    if (state.state == RequesterStateKind::Requesting) {
      const std::uint64_t age_ns =
          (now >= state.request_sent_ns) ? now - state.request_sent_ns : 0;
      if (age_ns > request_timeout_ns) {
        RCLCPP_WARN(get_logger(),
            "Negotiation request timed out | peer_id=%s; request_id=%s; "
            "age_sec=%.2f; timeout_sec=%.2f",
            peer_id.c_str(),
            state.in_flight_request_id
                ? state.in_flight_request_id->c_str() : "(none)",
            static_cast<double>(age_ns) / 1e9, request_timeout_sec_);

        state.state = RequesterStateKind::Idle;
        state.in_flight_request_id.reset();
        state.request_sent_ns = 0;
        state.proposed_own_claims.clear();
        state.proposed_peer_claims.clear();
        state.backoff_until_ns =
            now + static_cast<std::uint64_t>(request_timeout_backoff_sec_ * 1e9);
      }
      continue;  // only check timeouts while in REQUESTING state
    }

    if (state.state != RequesterStateKind::Idle) {
      RCLCPP_WARN(get_logger(),
          "Unknown requester state for peer_id=%s; resetting",
          peer_id.c_str());
      state = RequesterState{};
      continue;
    }

    if (now < state.backoff_until_ns) continue;

    if (last_negotiation_attempt_ns_ > 0) {
      const double cooldown_age_sec =
          static_cast<double>(now - last_negotiation_attempt_ns_) / 1e9;
      if (cooldown_age_sec < negotiation_cooldown_sec_) continue;
    }

    auto proposal = build_negotiation_proposal(peer_id, candidate_frontiers);
    if (!proposal) continue;

    const auto & [proposed_own_claims, proposed_peer_claims] = *proposal;

    // Suppression: avoid repeatedly negotiating the same already-
    // committed allocation.
    const auto peer_claims_it = peer_claims_.find(peer_id);
    const std::vector<ClaimedFrontier> current_peer_claims =
        peer_claims_it != peer_claims_.end()
            ? peer_claims_it->second
            : std::vector<ClaimedFrontier>{};
    if (claims_match_within_tolerance(proposed_own_claims, own_claims_) &&
        claims_match_within_tolerance(proposed_peer_claims, current_peer_claims)) {
      continue;
    }

    const std::string request_id = next_request_id();

    NegotiationRequest req;
    req.header = make_header();
    req.request_id = request_id;
    req.requester_id = robot_id_;
    req.responder_id = peer_id;
    req.request_stamp = ns_to_time_msg(now_ns());
    req.requester_claims = proposed_own_claims;
    req.responder_claims = proposed_peer_claims;
    req.protocol_version = kProtocolVersion;

    peer_request_pubs_.at(peer_id)->publish(req);

    state.state = RequesterStateKind::Requesting;
    state.in_flight_request_id = request_id;
    state.request_sent_ns = now;
    state.proposed_own_claims = proposed_own_claims;
    state.proposed_peer_claims = proposed_peer_claims;

    last_negotiation_attempt_ns_ = now;
    last_interaction_attempt_ns_ = now;

    RCLCPP_INFO(get_logger(),
        "NegotiationRequest sent | peer_id=%s; request_id=%s; "
        "own_claims=%zu; peer_claims=%zu",
        peer_id.c_str(), request_id.c_str(),
        proposed_own_claims.size(), proposed_peer_claims.size());
  }
}

}  // namespace cfpa2_peer_coordination