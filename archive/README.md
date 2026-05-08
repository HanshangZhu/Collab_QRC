# archive/

Preserved ROS 2 code, removed from active build in `native/no-ros2` branch.
All algorithm logic has been extracted and lives under `src/native/`.
Nothing here is deleted — git history preserves full context.

## Layout

```
archive/
  real_robot/           Real Go2W/Go2 bringup + ROS 2 drivers + real-robot scripts
    go2w_real_bringup/  Launch files, SLAM configs, RViz configs, calibration tools
    unitree_go2w_ros2/  Unitree ROS 2 SDK (msgs, srvs, driver node)
    real/               Shell scripts: connect, calibrate, monitor, deploy
    fastdds_no_shm.xml  DDS profile (FastDDS, shared-memory disabled)

  ros2_legacy/          Simulation + nav ROS 2 infrastructure, now replaced natively
    mujoco_ros2_control/  DFKI MuJoCo hardware interface plugin (C++)
    fast_lio/             Fast-LIO2 SLAM (ROS 2 Humble)
    sc_pgo/               Scan-context pose graph optimizer
    livox_ros_driver2/    Livox LiDAR ROS 2 driver
    Livox-SDK2/           Livox C++ SDK
    champ/                CHAMP quadruped IK / gait controller
    go2_gazebo_sim_launch/  All ROS launch files for MuJoCo/Gazebo sim
    go2w_control/         Locomotion ROS 2 package
    go2w_perception/      Perception bridge ROS 2 package
    go2w_safety/          Safety-filter ROS 2 node
    go2w_observability/   Metrics logger ROS 2 node
    go2w_spawn/           Robot spawner ROS 2 package
    mujoco_sensor_bridge/ MuJoCo sensor→ROS bridge
    go2w_config/          Nav2 YAML configs + navigation launch
    scripts_runtime/      ROS 2 nodes started by launch files
    scripts_bench/        Multi-trial benchmark runners
    scripts_launch/       User-invoked ROS launch entry points
    scripts_ops/          One-shot ops & dev utilities

  docker/               Docker setup for ROS 2 on macOS (Ubuntu 22.04 container)
```

## How to revive

Restore any path with:
```bash
git mv archive/<path> <original-path>
git checkout native/no-ros2   # or desired branch
colcon build --symlink-install --packages-select <pkg>
```

The last fully-working ROS 2 commit is the parent of the first commit on `native/no-ros2`.
