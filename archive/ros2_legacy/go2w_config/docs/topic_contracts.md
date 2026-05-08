# Subsystem Interface Contracts

All topics below are **relative** (no leading `/`).
Namespace is set by the launch file via `PushRosNamespace` or `namespace=`.

## Perception (go2w_perception)

**Publishes:**
| Topic | Type | Notes |
|-------|------|-------|
| `odom/nav` | nav_msgs/Odometry | Fused SLAM odometry (from slam_odom_relay or carto_odom_bridge) |
| `registered_scan_reliable` | sensor_msgs/PointCloud2 | QoS-bridged registered scan (RELIABLE) |
| `scan_3d` | sensor_msgs/LaserScan | 2D projected scan (from pointcloud_to_laserscan) |

**TF:** `map` -> `odom`

**Subscribes:**
| Topic | Type |
|-------|------|
| `imu/data` | sensor_msgs/Imu |
| `registered_scan` | sensor_msgs/PointCloud2 (BEST_EFFORT) |

## Navigation (go2w_nav + cfpa2)

**Publishes:**
| Topic | Type | Notes |
|-------|------|-------|
| `cmd_vel_stamped` | geometry_msgs/TwistStamped | Reactive nav velocity output |
| `planned_path` | nav_msgs/Path | Current planned path |
| `nav_status` | std_msgs/String | Navigation state |
| `way_point_coord` | geometry_msgs/PointStamped | From CFPA2 frontier planner |
| `map` | nav_msgs/OccupancyGrid | From simple_scan_mapper |

**Subscribes:**
| Topic | Type |
|-------|------|
| `odom/nav` | nav_msgs/Odometry |
| `scan_3d` | sensor_msgs/LaserScan |
| `way_point_coord` | geometry_msgs/PointStamped |
| `stop` | std_msgs/Int8 |

## Safety (go2w_safety)

**Publishes:**
| Topic | Type | Notes |
|-------|------|-------|
| `stop` | std_msgs/Int8 | 1 = wall detected, 0 = clear |
| `joy` | sensor_msgs/Joy | Synthetic joystick for autonomy enablement |

**Subscribes:**
| Topic | Type |
|-------|------|
| `scan_3d` | sensor_msgs/LaserScan |
| `way_point_coord` | geometry_msgs/PointStamped |

## Control Routing (go2w_control)

**Publishes:**
| Topic | Type | Notes |
|-------|------|-------|
| `cmd_vel` | geometry_msgs/Twist | Final muxed velocity |
| wheel controller commands | Float64MultiArray | Wheel velocity controller |

**Subscribes:**
| Topic | Type |
|-------|------|
| `cmd_vel_stamped` | geometry_msgs/TwistStamped |
| `mobility_mode` | std_msgs/String |

## Observability (go2w_observability)

**Subscribes only** (does not affect robot behavior):
| Topic | Type |
|-------|------|
| `odom/nav` | nav_msgs/Odometry |
| `map` | nav_msgs/OccupancyGrid |
| `nav_status` | std_msgs/String |

## Cross-Robot Topics

Teammate odometry for multi-robot avoidance is parameterized:
```yaml
# In default_nav config:
teammate_odom_topics: ["/robot_b/odom/nav"]
```
Never hardcoded in node source code.
