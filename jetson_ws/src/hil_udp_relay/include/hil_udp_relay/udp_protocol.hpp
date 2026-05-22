// udp_protocol.hpp — HIL UDP relay wire protocol (laptop ROS 2 ⇄ NX ROS 1).
//
// WHY THIS EXISTS
//   The standard ros1_bridge is broken on the Orin NX (Foxy EOL → bad_alloc on
//   every message). This file defines a hand-rolled, typed UDP wire protocol so
//   we can shuttle a fixed set of topics between a ROS 2 Humble laptop and a
//   ROS 1 Noetic Jetson without ros1_bridge. It is ROS-distro-agnostic on
//   purpose — pure POD packing, no ROS headers — so the SAME file can be copied
//   verbatim into both the colcon (ROS 2) and catkin (ROS 1) packages. The two
//   copies MUST stay byte-for-byte identical or rx/tx will desync.
//
// KEY ASSUMPTIONS / GOTCHAS
//   * ENDIANNESS: both ends are little-endian (x86_64 laptop + aarch64 Orin NX).
//     We raw-memcpy POD fields into the byte buffer with no host↔network byte
//     swapping. If a big-endian host ever joins the bench this breaks — every
//     get_*/put_* helper would need an explicit byte-swap. Documented, not
//     guarded, by design (the bench is fixed-arch).
//   * ROS 1 and ROS 2 wire serialization formats DIFFER, so we never reuse
//     ROS's own (de)serialization. Each msg type is packed field-by-field here.
//   * FRAGMENTATION: a single UDP datagram maxes out near 65507 bytes. A Livox
//     CustomMsg frame is ~270 KB, a PointCloud2/OccupancyGrid can be larger. We
//     split any payload over kMaxFragPayload (~60000) into fragments, each
//     prefixed with the same PacketHeader but carrying frag_index/frag_count and
//     a shared seq. The rx side reassembles by seq and delivers once all
//     fragments arrive; incomplete sets are dropped when a newer seq arrives or
//     a staleness timeout elapses.
//
// The serializers below append to / read from a std::vector<uint8_t>. They use a
// little manual cursor (Writer / Reader) so the layout is explicit and auditable.

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace hil_udp_relay {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// Magic word identifying our packets ("HUDP" little-endian).
constexpr uint32_t kMagic = 0x50445548u;  // 'H' 'U' 'D' 'P'

// Max application payload bytes per UDP datagram BEFORE the PacketHeader is
// prepended. Conservative under the ~65507 IPv4/UDP datagram ceiling, leaving
// room for the header and any path MTU surprises (the OS still fragments at the
// IP layer, but keeping datagrams modest reduces whole-message loss).
constexpr uint32_t kMaxFragPayload = 60000u;

// Message type tags. Stable wire values — do not renumber.
enum MsgType : uint16_t {
  MSG_LIDAR_CUSTOM = 1,  // livox_ros_driver2 CustomMsg
  MSG_IMU = 2,           // sensor_msgs/Imu
  MSG_TWIST = 3,         // geometry_msgs/Twist
  MSG_ODOM = 4,          // nav_msgs/Odometry
  MSG_OCCGRID = 5,       // nav_msgs/OccupancyGrid
  MSG_POINTSTAMPED = 6,  // geometry_msgs/PointStamped
  MSG_POSESTAMPED = 7,   // geometry_msgs/PoseStamped
  MSG_PATH = 8,          // nav_msgs/Path
};

// Packet header. PACKED so its on-wire size is exactly 24 bytes regardless of
// compiler padding. Every datagram (fragment) starts with this.
#pragma pack(push, 1)
struct PacketHeader {
  uint32_t magic;        // == kMagic
  uint16_t msg_type;     // MsgType
  uint16_t flags;        // reserved (0)
  uint32_t seq;          // per-(tx,msg_type) monotonically increasing message id
  uint32_t total_len;    // total application payload length (all fragments)
  uint32_t frag_offset;  // byte offset of this fragment within total payload
  uint16_t frag_index;   // 0..frag_count-1
  uint16_t frag_count;   // number of fragments for this message
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 24, "PacketHeader must be 24 bytes on wire");

constexpr size_t kHeaderSize = sizeof(PacketHeader);

// ---------------------------------------------------------------------------
// POD-typed neutral message structs
//
// These mirror the ROS message fields but carry NO ROS dependency, so the
// serializers compile in both the ROS 1 and ROS 2 packages. Each ROS node
// converts its native msg <-> these structs at the boundary.
// ---------------------------------------------------------------------------

struct Header {
  uint32_t stamp_sec = 0;
  uint32_t stamp_nsec = 0;
  std::string frame_id;
};

struct CustomPoint {
  uint32_t offset_time = 0;
  float x = 0.f, y = 0.f, z = 0.f;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
};

struct CustomMsg {
  Header header;
  uint64_t timebase = 0;
  uint32_t point_num = 0;
  uint8_t lidar_id = 0;
  uint8_t rsvd[3] = {0, 0, 0};
  std::vector<CustomPoint> points;
};

struct Imu {
  Header header;
  double orientation[4] = {0, 0, 0, 1};       // x, y, z, w
  double angular_velocity[3] = {0, 0, 0};     // x, y, z
  double linear_acceleration[3] = {0, 0, 0};  // x, y, z
};

struct Twist {
  double linear[3] = {0, 0, 0};   // x, y, z
  double angular[3] = {0, 0, 0};  // x, y, z
};

struct Odometry {
  Header header;
  std::string child_frame_id;
  double position[3] = {0, 0, 0};      // x, y, z
  double orientation[4] = {0, 0, 0, 1};  // x, y, z, w
  Twist twist;
};

struct MapMetaData {
  float resolution = 0.f;
  uint32_t width = 0;
  uint32_t height = 0;
  double origin_position[3] = {0, 0, 0};
  double origin_orientation[4] = {0, 0, 0, 1};
};

struct OccupancyGrid {
  Header header;
  MapMetaData info;
  std::vector<int8_t> data;
};

struct PointStamped {
  Header header;
  double point[3] = {0, 0, 0};
};

struct PoseStamped {
  Header header;
  double position[3] = {0, 0, 0};
  double orientation[4] = {0, 0, 0, 1};
};

struct Path {
  Header header;
  std::vector<PoseStamped> poses;
};

// ---------------------------------------------------------------------------
// Little-endian byte writer / reader (raw memcpy; LE-only — see file header)
// ---------------------------------------------------------------------------

class Writer {
 public:
  explicit Writer(std::vector<uint8_t>& buf) : buf_(buf) {}

  template <typename T>
  void pod(const T& v) {
    static_assert(std::is_trivially_copyable<T>::value, "pod requires trivial type");
    const auto* p = reinterpret_cast<const uint8_t*>(&v);
    buf_.insert(buf_.end(), p, p + sizeof(T));
  }

  void u8(uint8_t v) { buf_.push_back(v); }
  void u16(uint16_t v) { pod(v); }
  void u32(uint32_t v) { pod(v); }
  void u64(uint64_t v) { pod(v); }
  void f32(float v) { pod(v); }
  void f64(double v) { pod(v); }
  void i8(int8_t v) { pod(v); }

  // Length-prefixed string: u32 length + raw bytes.
  void str(const std::string& s) {
    u32(static_cast<uint32_t>(s.size()));
    buf_.insert(buf_.end(), s.begin(), s.end());
  }

  // Raw byte blob, no length prefix (caller wrote the count separately).
  void bytes(const void* p, size_t n) {
    const auto* b = reinterpret_cast<const uint8_t*>(p);
    buf_.insert(buf_.end(), b, b + n);
  }

 private:
  std::vector<uint8_t>& buf_;
};

class Reader {
 public:
  Reader(const uint8_t* data, size_t len) : p_(data), end_(data + len) {}

  bool ok() const { return !bad_; }
  size_t remaining() const { return static_cast<size_t>(end_ - p_); }

  template <typename T>
  T pod() {
    static_assert(std::is_trivially_copyable<T>::value, "pod requires trivial type");
    T v{};
    if (remaining() < sizeof(T)) {
      bad_ = true;
      return v;
    }
    std::memcpy(&v, p_, sizeof(T));
    p_ += sizeof(T);
    return v;
  }

  uint8_t u8() { return pod<uint8_t>(); }
  uint16_t u16() { return pod<uint16_t>(); }
  uint32_t u32() { return pod<uint32_t>(); }
  uint64_t u64() { return pod<uint64_t>(); }
  float f32() { return pod<float>(); }
  double f64() { return pod<double>(); }
  int8_t i8() { return pod<int8_t>(); }

  std::string str() {
    uint32_t n = u32();
    if (bad_ || remaining() < n) {
      bad_ = true;
      return {};
    }
    std::string s(reinterpret_cast<const char*>(p_), n);
    p_ += n;
    return s;
  }

  // Copy n raw bytes into dst. Bounds-checked.
  void bytes(void* dst, size_t n) {
    if (remaining() < n) {
      bad_ = true;
      return;
    }
    std::memcpy(dst, p_, n);
    p_ += n;
  }

 private:
  const uint8_t* p_;
  const uint8_t* end_;
  bool bad_ = false;
};

// ---------------------------------------------------------------------------
// Per-type serialize / deserialize. Each serialize_* appends the full message
// payload to `out`; deserialize_* reads from a Reader and returns success.
// ---------------------------------------------------------------------------

inline void serialize_header(Writer& w, const Header& h) {
  w.u32(h.stamp_sec);
  w.u32(h.stamp_nsec);
  w.str(h.frame_id);
}

inline void deserialize_header(Reader& r, Header& h) {
  h.stamp_sec = r.u32();
  h.stamp_nsec = r.u32();
  h.frame_id = r.str();
}

// --- CustomMsg ---
inline void serialize_custommsg(Writer& w, const CustomMsg& m) {
  serialize_header(w, m.header);
  w.u64(m.timebase);
  w.u32(m.point_num);
  w.u8(m.lidar_id);
  w.u8(m.rsvd[0]);
  w.u8(m.rsvd[1]);
  w.u8(m.rsvd[2]);
  // Count of points actually packed (authoritative for the reader; point_num is
  // the Livox-reported count and should match points.size()).
  w.u32(static_cast<uint32_t>(m.points.size()));
  for (const auto& p : m.points) {
    w.u32(p.offset_time);
    w.f32(p.x);
    w.f32(p.y);
    w.f32(p.z);
    w.u8(p.reflectivity);
    w.u8(p.tag);
    w.u8(p.line);
  }
}

inline bool deserialize_custommsg(Reader& r, CustomMsg& m) {
  deserialize_header(r, m.header);
  m.timebase = r.u64();
  m.point_num = r.u32();
  m.lidar_id = r.u8();
  m.rsvd[0] = r.u8();
  m.rsvd[1] = r.u8();
  m.rsvd[2] = r.u8();
  uint32_t n = r.u32();
  if (!r.ok()) return false;
  m.points.clear();
  m.points.reserve(n);
  for (uint32_t i = 0; i < n; ++i) {
    CustomPoint p;
    p.offset_time = r.u32();
    p.x = r.f32();
    p.y = r.f32();
    p.z = r.f32();
    p.reflectivity = r.u8();
    p.tag = r.u8();
    p.line = r.u8();
    if (!r.ok()) return false;
    m.points.push_back(p);
  }
  return r.ok();
}

// --- Imu ---
inline void serialize_imu(Writer& w, const Imu& m) {
  serialize_header(w, m.header);
  for (int i = 0; i < 4; ++i) w.f64(m.orientation[i]);
  for (int i = 0; i < 3; ++i) w.f64(m.angular_velocity[i]);
  for (int i = 0; i < 3; ++i) w.f64(m.linear_acceleration[i]);
}

inline bool deserialize_imu(Reader& r, Imu& m) {
  deserialize_header(r, m.header);
  for (int i = 0; i < 4; ++i) m.orientation[i] = r.f64();
  for (int i = 0; i < 3; ++i) m.angular_velocity[i] = r.f64();
  for (int i = 0; i < 3; ++i) m.linear_acceleration[i] = r.f64();
  return r.ok();
}

// --- Twist ---
inline void serialize_twist(Writer& w, const Twist& m) {
  for (int i = 0; i < 3; ++i) w.f64(m.linear[i]);
  for (int i = 0; i < 3; ++i) w.f64(m.angular[i]);
}

inline bool deserialize_twist(Reader& r, Twist& m) {
  for (int i = 0; i < 3; ++i) m.linear[i] = r.f64();
  for (int i = 0; i < 3; ++i) m.angular[i] = r.f64();
  return r.ok();
}

// --- Odometry ---
inline void serialize_odom(Writer& w, const Odometry& m) {
  serialize_header(w, m.header);
  w.str(m.child_frame_id);
  for (int i = 0; i < 3; ++i) w.f64(m.position[i]);
  for (int i = 0; i < 4; ++i) w.f64(m.orientation[i]);
  serialize_twist(w, m.twist);
}

inline bool deserialize_odom(Reader& r, Odometry& m) {
  deserialize_header(r, m.header);
  m.child_frame_id = r.str();
  for (int i = 0; i < 3; ++i) m.position[i] = r.f64();
  for (int i = 0; i < 4; ++i) m.orientation[i] = r.f64();
  deserialize_twist(r, m.twist);
  return r.ok();
}

// --- OccupancyGrid ---
inline void serialize_occgrid(Writer& w, const OccupancyGrid& m) {
  serialize_header(w, m.header);
  w.f32(m.info.resolution);
  w.u32(m.info.width);
  w.u32(m.info.height);
  for (int i = 0; i < 3; ++i) w.f64(m.info.origin_position[i]);
  for (int i = 0; i < 4; ++i) w.f64(m.info.origin_orientation[i]);
  w.u32(static_cast<uint32_t>(m.data.size()));
  if (!m.data.empty()) w.bytes(m.data.data(), m.data.size());
}

inline bool deserialize_occgrid(Reader& r, OccupancyGrid& m) {
  deserialize_header(r, m.header);
  m.info.resolution = r.f32();
  m.info.width = r.u32();
  m.info.height = r.u32();
  for (int i = 0; i < 3; ++i) m.info.origin_position[i] = r.f64();
  for (int i = 0; i < 4; ++i) m.info.origin_orientation[i] = r.f64();
  uint32_t n = r.u32();
  if (!r.ok()) return false;
  m.data.resize(n);
  if (n > 0) r.bytes(m.data.data(), n);
  return r.ok();
}

// --- PointStamped ---
inline void serialize_pointstamped(Writer& w, const PointStamped& m) {
  serialize_header(w, m.header);
  for (int i = 0; i < 3; ++i) w.f64(m.point[i]);
}

inline bool deserialize_pointstamped(Reader& r, PointStamped& m) {
  deserialize_header(r, m.header);
  for (int i = 0; i < 3; ++i) m.point[i] = r.f64();
  return r.ok();
}

// --- PoseStamped ---
inline void serialize_posestamped(Writer& w, const PoseStamped& m) {
  serialize_header(w, m.header);
  for (int i = 0; i < 3; ++i) w.f64(m.position[i]);
  for (int i = 0; i < 4; ++i) w.f64(m.orientation[i]);
}

inline bool deserialize_posestamped(Reader& r, PoseStamped& m) {
  deserialize_header(r, m.header);
  for (int i = 0; i < 3; ++i) m.position[i] = r.f64();
  for (int i = 0; i < 4; ++i) m.orientation[i] = r.f64();
  return r.ok();
}

// --- Path ---
inline void serialize_path(Writer& w, const Path& m) {
  serialize_header(w, m.header);
  w.u32(static_cast<uint32_t>(m.poses.size()));
  for (const auto& pose : m.poses) {
    serialize_posestamped(w, pose);
  }
}

inline bool deserialize_path(Reader& r, Path& m) {
  deserialize_header(r, m.header);
  uint32_t n = r.u32();
  if (!r.ok()) return false;
  m.poses.clear();
  m.poses.reserve(n);
  for (uint32_t i = 0; i < n; ++i) {
    PoseStamped pose;
    if (!deserialize_posestamped(r, pose)) return false;
    m.poses.push_back(pose);
  }
  return r.ok();
}

}  // namespace hil_udp_relay
