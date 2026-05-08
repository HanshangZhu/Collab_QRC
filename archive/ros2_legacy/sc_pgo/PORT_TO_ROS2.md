# Porting `sc_pgo` (engcang/FAST-LIO-SAM) to ROS 2 Humble

## Why this folder exists

`src/vendor/sc_pgo/` holds vendored sources for a Fast-LIO2 loop-closure
post-processor. The launch file
`src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py`
exposes a `loop_closure:=true|false` toggle that, when `true`, attempts to
spawn a `sc_pgo_node` from the `sc_pgo` package and feed
`/<ns>/corrected_odom` into `slam_odom_relay`. Until the port below is
done, the toggle silently warns and skips — Fast-LIO2 runs open-loop
(pure ICP scan-matching, drifts ~10 m over 7 minutes on demo3_mixed).

## What's vendored

`fast_lio_sam/` is from `engcang/FAST-LIO-SAM` (commit clone, ROS 1
catkin). The PGO module is modular per the upstream README:

> Note2: main code (PGO) is modularized and hence can be combined with
> any other LIO / LO

The repo is `COLCON_IGNORE`-d so colcon skips it during build (otherwise
the catkin `package.xml` would fail).

## Topic interface expected by launch

When (eventually) built and `loop_closure:=true`, the node is launched
with these remappings — match these in your port:

| Direction | Standard topic         | Remap target                     |
|-----------|------------------------|----------------------------------|
| Subscribe | `/aft_mapped_to_init`  | `/<ns>/Odometry`                 |
| Subscribe | `/cloud_registered`    | `/<ns>/cloud_registered_body`    |
| Publish   | `/corrected_odom`      | `/<ns>/corrected_odom`           |
| Publish   | `/corrected_path`      | `/<ns>/corrected_path`           |
| Publish   | `/corrected_cloud`     | `/<ns>/corrected_cloud`          |
| Publish   | `/corrected_map`       | `/<ns>/corrected_map`            |

`slam_odom_relay` already prefers `/<ns>/corrected_odom` when fresh,
falls back to `/<ns>/Odometry` when stale (see
`src/go2w/go2w_perception/scripts/slam_odom_relay.py:127-144`).

## Port checklist

1. **Rename or keep?** Launch expects `package="sc_pgo"` /
   `executable="sc_pgo_node"`. Either:
   - Rename `fast_lio_sam/` directory to `sc_pgo/`, set
     `<name>sc_pgo</name>` in `package.xml`, rename the executable target
     in `CMakeLists.txt` to `sc_pgo_node`; OR
   - Keep `fast_lio_sam` and edit the launch to match.
2. **`package.xml`**:
   ```xml
   <export><build_type>ament_cmake</build_type></export>
   <buildtool_depend>ament_cmake</buildtool_depend>
   <depend>rclcpp</depend>
   <depend>nav_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>tf2</depend>
   <depend>tf2_ros</depend>
   <depend>tf2_geometry_msgs</depend>
   <depend>pcl_ros</depend>
   <depend>pcl_conversions</depend>
   ```
3. **`CMakeLists.txt`**: switch from `catkin_package` /
   `add_dependencies(... ${catkin_EXPORTED_TARGETS})` to
   `find_package(rclcpp REQUIRED)` etc., `ament_target_dependencies(...)`,
   `ament_package()`, `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})`.
4. **GTSAM**: upstream uses `gtsam`; install `libgtsam-dev` from apt
   (`sudo apt install ros-humble-gtsam` on humble) or vendor it under
   `third_party/`.
5. **API porting (mostly mechanical)**:
   - `ros::NodeHandle nh` → `auto node = std::make_shared<rclcpp::Node>("sc_pgo")`
   - `nh.subscribe<Odometry>("topic", 10, cb)` →
     `node->create_subscription<Odometry>("topic", 10, cb)`
   - `nh.advertise<Odometry>("topic", 10)` →
     `node->create_publisher<Odometry>("topic", 10)`
   - `ros::Time::now()` → `node->get_clock()->now()`
   - `ros::Rate r(10); r.sleep()` → `rclcpp::Rate r(10); r.sleep()`
   - Spinner: replace `ros::AsyncSpinner` with
     `rclcpp::executors::MultiThreadedExecutor`
   - Log macros: `ROS_INFO(...)` → `RCLCPP_INFO(node->get_logger(), ...)`
6. **Frame conventions**: upstream publishes in `map` frame. Verify
   `slam_odom_relay`'s `output_frame_id` (currently `world`) matches
   what your port outputs, or convert in `relay_cb`.
7. **Tighter LC threshold** (per user request 2026-04-29 "调严格"):
   the upstream radius-search default is 5.0 m + ICP fitness 0.3.
   Tighten to 3.0 m radius + ICP fitness 0.15 for indoor `demo3_mixed`
   to avoid false positives like the one we hypothesised earlier.

## Quick smoke test once ported

```bash
colcon build --packages-select sc_pgo
ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  nav_backend_a:=nav2_mppi nav_backend_b:=none \
  loop_closure:=true gui:=true rviz:=true
```

Expected: `[robot_a.sc_pgo]` log lines appear; once a loop is detected,
`/robot_a/corrected_odom` starts publishing. Verify with the drift
monitor used 2026-04-29:

```python
GT vs corrected drift: should converge towards 0 after the first loop
closure. Without sc_pgo: drift accumulates ~10 m on long demo3 runs.
```

## Alternatives if porting feels too heavy

- Switch SLAM to `koide3/glim_ros2` (built-in PGO) — replaces Fast-LIO,
  bigger refactor but ROS 2 native.
- Use `gisbi-kim/scancontext` (the SC descriptor lib alone, easier to
  embed) and write a thin custom ROS 2 PGO node around GTSAM.
