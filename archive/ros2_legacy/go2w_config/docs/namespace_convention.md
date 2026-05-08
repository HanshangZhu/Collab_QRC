# Namespace Convention

## Rules

1. **Nodes use relative topic names only** (no leading `/`).
   The ROS2 namespace mechanism automatically prefixes `/{ns}/`.

2. **Remappings are for interface adaptation only**, not for namespacing.
   Valid: `("/Odometry", "odom/slam")` (Fast-LIO output → our convention).
   Invalid: `("/way_point", f"/{robot_ns}/way_point_coord")` (namespace patching).

3. **Cross-robot topics are parameters**, not hardcoded if/elif chains.
   ```python
   self.declare_parameter("teammate_odom_topics", [""])
   ```

4. **Sub-launches inherit namespace via PushRosNamespace**.
   ```python
   GroupAction([
       PushRosNamespace("robot_a"),
       IncludeLaunchDescription("perception.launch.py"),
   ])
   ```

## Current State (pre-migration)

Many nodes still use absolute topic names with launch-level remapping
to compensate. This convention documents the target state.
Migration will happen incrementally as nodes are touched.

## Multi-Robot

- Single robot: `namespace=robot`
- Dual robot: `namespace=robot_a`, `namespace=robot_b`
- Adding robot_c: one new `GroupAction` block, one extra entry in
  `teammate_odom_topics` parameter. No node code changes.
