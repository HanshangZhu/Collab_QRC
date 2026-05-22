// peer_coordinator.hpp — Decentralised peer coordinator node.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cfpa2_peer_coordination_msgs/msg/claimed_frontier.hpp"
#include "cfpa2_peer_coordination_msgs/msg/negotiation_request.hpp"
#include "cfpa2_peer_coordination_msgs/msg/negotiation_response.hpp"
#include "cfpa2_peer_coordination_msgs/msg/peer_state.hpp"

#include "cfpa2_peer_coordination/mdvrp_adapter.hpp"  // Point3 + helpers

namespace cfpa2_peer_coordination {

// ── Message type aliases. Point3 is brought in from mdvrp_adapter.hpp
// so it stays defined in one place, mirroring the Python module split.

using ClaimedFrontier = cfpa2_peer_coordination_msgs::msg::ClaimedFrontier;
using NegotiationRequest = cfpa2_peer_coordination_msgs::msg::NegotiationRequest;
using NegotiationResponse = cfpa2_peer_coordination_msgs::msg::NegotiationResponse;
using PeerState = cfpa2_peer_coordination_msgs::msg::PeerState;

// ── Protocol constants. Both peers MUST agree on these for claim
// equivalence to behave the same on both sides of a negotiation.
constexpr std::uint8_t kProtocolVersion = 1;
constexpr double kFrontierMatchToleranceM = 0.5;

// Requester state-machine kinds. Internal only — never serialised.
enum class RequesterStateKind {
  Idle,
  Requesting,
};

// Negotiation responder outcomes. These strings ARE serialised onto
// NegotiationResponse.reason, so the values must match the Python
// implementation exactly. Both peers parse them.
namespace reject_reason {
constexpr const char * kAccepted = "accepted";
constexpr const char * kCrossingLost = "crossing_lost";
constexpr const char * kLocalSolveMismatch = "local_solve_mismatch";
constexpr const char * kClaimOverlap = "claim_overlap";
constexpr const char * kFrontierUnknownLocally = "frontier_unknown_locally";
constexpr const char * kProtocolError = "protocol_error";
constexpr const char * kNoLocalExpectedProposal = "no_local_expected_proposal";
}  // namespace reject_reason

// Per-peer state held locally for staleness tracking + last-received
// snapshot. last_state is empty until the first PeerState arrives.
struct PeerInfo {
  std::optional<PeerState> last_state;
  std::uint64_t last_received_ns = 0;
  bool was_fresh = false;
};

// Per-peer requester-side negotiation state. Tracks one in-flight
// proposal from this robot to a specific peer. Claims are not
// committed until the peer accepts the request.
struct RequesterState {
  RequesterStateKind state = RequesterStateKind::Idle;
  std::optional<std::string> in_flight_request_id;
  std::uint64_t request_sent_ns = 0;
  std::vector<ClaimedFrontier> proposed_own_claims;
  std::vector<ClaimedFrontier> proposed_peer_claims;
  int consecutive_rejects = 0;
  std::uint64_t backoff_until_ns = 0;
};

class PeerCoordinatorNode : public rclcpp::Node
{
public:
  explicit PeerCoordinatorNode(
      const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ~PeerCoordinatorNode() override = default;

private:
  // ─── parameter values, cached at startup ─────────────────────────
  std::string robot_id_;
  std::string robot_namespace_;
  std::vector<std::string> peer_namespaces_;
  std::vector<std::string> peer_ids_;  // derived view: == peer_namespaces_

  double peer_timeout_sec_ = 0.0;
  double claim_timeout_sec_ = 0.0;
  double peer_state_rate_hz_ = 0.0;
  double negotiation_rate_hz_ = 0.0;
  double negotiation_cooldown_sec_ = 0.0;

  double request_timeout_sec_ = 0.0;
  double request_reject_backoff_sec_ = 0.0;
  double request_timeout_backoff_sec_ = 0.0;
  bool enable_negotiation_requests_ = true;

  std::string odom_topic_suffix_;
  std::string frontier_markers_topic_;
  double blocked_frontiers_rate_hz_ = 0.0;

  double mdvrp_time_limit_sec_ = 0.0;
  int mdvrp_span_cost_coefficient_ = 0;

  // ─── topic name cache (built from parameters) ────────────────────
  std::string own_peer_state_topic_;
  std::string own_blocked_frontiers_topic_;
  std::string own_request_inbox_topic_;
  std::string own_response_inbox_topic_;
  std::string own_odom_topic_;

  std::unordered_map<std::string, std::string> peer_state_topics_;
  std::unordered_map<std::string, std::string> peer_request_inbox_topics_;
  std::unordered_map<std::string, std::string> peer_response_inbox_topics_;

  // ─── per-peer storage ────────────────────────────────────────────
  std::unordered_map<std::string, PeerInfo> peer_info_;
  std::unordered_map<std::string, RequesterState> requester_states_;
  std::unordered_map<std::string, std::vector<ClaimedFrontier>> peer_claims_;

  // ─── local state ─────────────────────────────────────────────────
  std::optional<geometry_msgs::msg::Pose> latest_pose_;
  std::vector<Point3> local_frontiers_;
  std::vector<ClaimedFrontier> own_claims_;

  std::uint64_t last_negotiation_attempt_ns_ = 0;
  std::uint64_t last_interaction_attempt_ns_ = 0;
  std::uint64_t last_successful_interaction_ns_ = 0;
  std::uint64_t request_counter_ = 0;

  bool logged_negotiation_stub_ = false;

  // ─── ROS plumbing ────────────────────────────────────────────────
  // Subscriptions stored heterogeneously — we only need the SharedPtrs
  // alive to keep callbacks active; the typed lambdas capture `this`.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  rclcpp::Publisher<PeerState>::SharedPtr peer_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blocked_frontiers_pub_;

  std::unordered_map<std::string, rclcpp::Publisher<NegotiationRequest>::SharedPtr>
      peer_request_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<NegotiationResponse>::SharedPtr>
      peer_response_pubs_;

  rclcpp::TimerBase::SharedPtr peer_state_timer_;
  rclcpp::TimerBase::SharedPtr negotiation_timer_;
  rclcpp::TimerBase::SharedPtr blocked_frontiers_timer_;

  // ─── parameter setup helpers ─────────────────────────────────────
  void declare_parameters();
  void read_parameters();
  void build_topic_names();
  void wire_pub_sub();
  void start_timers();
  void log_startup_banner();

  // ─── callbacks (subscriptions) ───────────────────────────────────
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_frontier_markers(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void on_peer_state(const PeerState::SharedPtr msg, const std::string & peer_id);
  void on_negotiation_request(const NegotiationRequest::SharedPtr msg);
  void on_negotiation_response(const NegotiationResponse::SharedPtr msg);

  // ─── timer ticks ─────────────────────────────────────────────────
  void publish_peer_state();
  void publish_blocked_frontiers();
  void decide_negotiation();

  // ─── helpers (implemented chunk-by-chunk) ────────────────────────
  std_msgs::msg::Header make_header();
  builtin_interfaces::msg::Time ns_to_time_msg(std::uint64_t ns) const;
  std::string next_request_id();
  std::uint64_t now_ns();

  // Claim/frontier helpers — added in later chunks.
  void expire_stale_claims();
  void handle_peer_staleness();
  void resolve_own_claim_conflicts();
  bool peer_is_fresh(const std::string & peer_id);
  std::vector<std::string> fresh_peer_ids();
  std::vector<std::string> stale_peer_ids();
  std::vector<Point3> available_local_frontiers();

  bool same_frontier_position(const Point3 & a, const Point3 & b) const;
  bool claim_is_fresh(const ClaimedFrontier & claim);
  std::uint64_t claim_stamp_ns(const ClaimedFrontier & claim) const;
  bool claim_wins_against(const ClaimedFrontier & a, const ClaimedFrontier & b) const;
  bool claims_match_within_tolerance(
      const std::vector<ClaimedFrontier> & a,
      const std::vector<ClaimedFrontier> & b) const;
  bool claim_position_known_locally(const ClaimedFrontier & claim) const;
  bool frontier_blocked_by_peer_claim(const Point3 & frontier) const;

  ClaimedFrontier point3_to_claim_for(
      const Point3 & point,
      const std::string & claimed_by,
      double information_gain = 0.0);

  // Negotiation logic — added in later chunks.
  std::optional<std::pair<std::vector<ClaimedFrontier>, std::vector<ClaimedFrontier>>>
      build_negotiation_proposal(
          const std::string & peer_id,
          const std::vector<Point3> & candidate_frontiers);

  std::optional<std::pair<std::vector<ClaimedFrontier>, std::vector<ClaimedFrontier>>>
      responder_local_resolve(const NegotiationRequest & msg);

  std::pair<bool, std::string> validate_negotiation_request(
      const NegotiationRequest & msg);

  void tick_requester_state(
      const std::vector<std::string> & fresh_peers,
      const std::vector<Point3> & candidate_frontiers);
};

}  // namespace cfpa2_peer_coordination


