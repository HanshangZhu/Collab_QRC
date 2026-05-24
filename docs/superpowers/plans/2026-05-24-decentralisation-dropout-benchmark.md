# Centralised vs Decentralised CFPA2 Dropout Benchmark — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure exploration-speed degradation vs inter-robot comms dropout {0,30,50,80%} for centralised (single coordinator) vs decentralised (peer-to-peer) CFPA2 on demo3_mixed.

**Architecture:** Reuse the existing coverage-over-time harness, made architecture-agnostic via a union-of-per-robot-maps coverage metric. Add a generic topic-interposition relay node that drops a configurable fraction of messages on the coordination links of whichever mode is active. Add a `coordination_mode` + `comms_dropout` launch arg pair, then a driver script that sweeps the 2×4×10 matrix and a summarizer that produces tables + plots.

**Tech Stack:** ROS 2 Humble (rclpy/rclcpp), MuJoCo sim, Python 3.10 (micromamba `cmu_env`), bash, matplotlib.

**Spec:** [docs/superpowers/specs/2026-05-24-decentralisation-dropout-benchmark-design.md](../specs/2026-05-24-decentralisation-dropout-benchmark-design.md)

**Branch:** `bench/decentralisation-dropout`

**Build/run prelude (run once per shell before ROS commands):**
```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Confirmed topics (from source reading):**
- Centralised coordinator (per ns): sub `/<ns>/map` (`nav_msgs/msg/OccupancyGrid`), sub `/<ns>/odom/nav` (`nav_msgs/msg/Odometry`), pub goal `/<ns>/way_point_coord` (`geometry_msgs/msg/PointStamped`).
- Decentralised peer_coordinator (per ns): pub `/<ns>/cfpa2_peer_coordination/peer_state` (`cfpa2_peer_coordination_msgs/msg/PeerState`, best_effort depth-1); pub to peer inbox `/<peer>/cfpa2_peer_coordination/inbox/negotiation_request` + `.../negotiation_response` (`cfpa2_peer_coordination_msgs/msg/NegotiationRequest` / `NegotiationResponse`, reliable); pub `/<ns>/cfpa2_peer_coordination/blocked_frontiers` (`geometry_msgs/msg/PoseArray`); sub own `/<ns>/odom/nav`; sub `frontier_markers_topic` (set explicitly per ns).
- single_robot consumes `/<ns>/cfpa2_peer_coordination/blocked_frontiers` (already wired, [cfpa2_single_robot.cpp:64](../../../src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_single_robot.cpp#L64)).

---

## Phase 0 — Coverage metric + decentralised bring-up (GATE)

### Task 1: Union-coverage pure helper + unit test

**Files:**
- Create: `src/go2w/go2w_observability/go2w_observability/coverage_util.py`
- Test: `src/go2w/go2w_observability/test/test_coverage_util.py`

- [ ] **Step 1: Write the failing test**

```python
# src/go2w/go2w_observability/test/test_coverage_util.py
from go2w_observability.coverage_util import union_known_cell_count


def test_union_counts_distinct_cells():
    a = {(0, 0), (1, 0), (2, 0)}
    b = {(2, 0), (3, 0)}
    # union = {(0,0),(1,0),(2,0),(3,0)} -> 4 distinct
    assert union_known_cell_count([a, b]) == 4


def test_union_empty_inputs_is_zero():
    assert union_known_cell_count([]) == 0
    assert union_known_cell_count([set(), set()]) == 0


def test_union_single_robot_passthrough():
    a = {(0, 0), (1, 1)}
    assert union_known_cell_count([a]) == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/go2w/go2w_observability && python -m pytest test/test_coverage_util.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'go2w_observability.coverage_util'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/go2w/go2w_observability/go2w_observability/coverage_util.py
"""Architecture-agnostic exploration-coverage helpers.

The union of per-robot known-cell sets gives a global coverage measure that
does not depend on any coordinator-only ``/merged_map`` topic, so the same
metric applies to both the centralised and decentralised CFPA2 stacks.
"""
from __future__ import annotations

from typing import Iterable


def union_known_cell_count(cell_sets: Iterable[set]) -> int:
    """Number of distinct (ix, iy) cells known by at least one robot."""
    sets = [s for s in cell_sets if s]
    if not sets:
        return 0
    return len(set().union(*sets))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/go2w/go2w_observability && python -m pytest test/test_coverage_util.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add src/go2w/go2w_observability/go2w_observability/coverage_util.py src/go2w/go2w_observability/test/test_coverage_util.py
git commit -m "feat(bench): architecture-agnostic union coverage helper"
```

---

### Task 2: Wire `global_coverage_source: union` into the metrics logger

**Files:**
- Modify: `src/go2w/go2w_observability/scripts/exploration_metrics_logger.py` (param block ~line 150; `_global_map_resolution` init ~line 186; the global-area computation ~line 760)

- [ ] **Step 1: Add the parameter (after the existing `global_map_topic` declaration, ~line 151)**

```python
        self.declare_parameter("global_coverage_source", "merged_map")  # merged_map | union
```

Read it next to where `self._global_map_topic` is read (~line 176):

```python
        self._global_coverage_source = str(
            self.get_parameter("global_coverage_source").value).strip()
```

- [ ] **Step 2: Import the helper near the top of the file**

```python
from go2w_observability.coverage_util import union_known_cell_count
```

- [ ] **Step 3: Compute global cells from the union when in union mode**

Replace the global-area line (~line 760):

```python
        global_area = self._global_map_known_cells * (self._global_map_resolution ** 2)
```

with:

```python
        if self._global_coverage_source == "union":
            cell_sets = [rs.explored_cells for rs in self.robots.values()]
            self._global_map_known_cells = union_known_cell_count(cell_sets)
            resolutions = [rs.map_resolution for rs in self.robots.values()
                           if rs.map_resolution > 0.0]
            if resolutions:
                self._global_map_resolution = resolutions[0]
        global_area = self._global_map_known_cells * (self._global_map_resolution ** 2)
```

- [ ] **Step 4: Make the `/merged_map` subscription conditional (so union mode doesn't require it)**

At the subscription guard (~line 282), change:

```python
        if self._global_map_topic:
```

to:

```python
        if self._global_map_topic and self._global_coverage_source == "merged_map":
```

- [ ] **Step 5: Verify the module imports cleanly under ROS**

Run:
```bash
micromamba run -n cmu_env python -c "import ast; ast.parse(open('src/go2w/go2w_observability/scripts/exploration_metrics_logger.py').read()); print('OK')"
```
Expected: `OK`

- [ ] **Step 6: Commit**

```bash
git add src/go2w/go2w_observability/scripts/exploration_metrics_logger.py
git commit -m "feat(bench): union global-coverage mode in exploration_metrics_logger"
```

---

### Task 3: Add `coordination_mode` to the mixed launch (decentralised branch)

**Files:**
- Modify: `src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py` (declare arg near other `DeclareLaunchArgument`s; CFPA2 node block — the section that creates `cfpa2_coordinator`)

**Context:** Today the launch unconditionally creates one `cfpa2_coordinator` Node with `namespaces:['robot_a','robot_b']`. Add a `coordination_mode` arg: `centralised` keeps that; `decentralised` instead creates, per namespace, a `cfpa2_single_robot_node[_cpp]` + a `peer_coordinator_node`, and creates NO central coordinator.

- [ ] **Step 1: Declare the argument**

Add alongside the other `DeclareLaunchArgument` entries:

```python
    DeclareLaunchArgument(
        "coordination_mode", default_value="centralised",
        description="centralised (single cfpa2_coordinator) | decentralised "
                    "(per-robot single_robot + peer_coordinator)"),
```

- [ ] **Step 2: Read it in the launch-setup function (where other LaunchConfigurations are resolved)**

```python
    coordination_mode = LaunchConfiguration("coordination_mode").perform(context)
```

- [ ] **Step 3: Guard the existing centralised coordinator creation**

Wrap the block that appends the `cfpa2_coordinator` Node so it only runs when centralised:

```python
    if coordination_mode == "centralised":
        # ... existing cfpa2_coordinator Node(...) creation + append ...
        pass
```

- [ ] **Step 4: Add the decentralised branch (per-namespace single_robot + peer_coordinator)**

```python
    if coordination_mode == "decentralised":
        cfpa2_suffix = LaunchConfiguration("cfpa2_executable_suffix").perform(context)  # "" or "_cpp"
        cfpa2_single_cfg = os.path.join(
            get_package_share_directory("cfpa2_collaborative_autonomy"),
            "config", "cfpa2_single_robot.yaml")
        robot_ns = ["robot_a", "robot_b"]
        for ns in robot_ns:
            peers = [p for p in robot_ns if p != ns]
            fm_topic = f"/{ns}/cfpa2/frontier_markers"
            nodes.append(Node(
                package="cfpa2_collaborative_autonomy",
                executable=f"cfpa2_single_robot_node{cfpa2_suffix}",
                name="cfpa2_single_robot",
                namespace=ns,
                parameters=[cfpa2_single_cfg, {
                    "use_sim_time": True,
                    "namespaces": [ns],
                    "frontier_markers_topic": fm_topic,
                }],
                output="screen",
            ))
            nodes.append(Node(
                package="cfpa2_peer_coordination",
                executable="peer_coordinator_node",
                name="peer_coordinator",
                namespace=ns,
                parameters=[{
                    "use_sim_time": True,
                    "robot_id": ns,
                    "robot_namespace": ns,
                    "peer_namespaces": peers,
                    "frontier_markers_topic": fm_topic,
                    "odom_topic_suffix": "/odom/nav",
                }],
                output="screen",
            ))
```

(Use the file's existing list name for accumulated actions if it is not `nodes` — match the surrounding code.)

- [ ] **Step 5: Syntax-check the launch file**

Run:
```bash
micromamba run -n cmu_env python -c "import ast; ast.parse(open('src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py').read()); print('OK')"
```
Expected: `OK`

- [ ] **Step 6: Commit**

```bash
git add src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py
git commit -m "feat(bench): coordination_mode arg (centralised|decentralised) in mixed launch"
```

---

### Task 4: Phase-0 GATE — decentralised dual actually explores at 0% dropout

**Files:** none created; this is a live verification + debugging task.

- [ ] **Step 1: Confirm the per-ns frontier-marker topic is published by single_robot**

Launch decentralised headless for ~60 s:
```bash
timeout 90 ./scripts/launch/nav_test_demo3_mixed.sh gui:=false rviz:=false \
  coordination_mode:=decentralised cfpa2_executable_suffix:=_cpp 2>&1 | tee /tmp/decentralised_phase0.log &
sleep 45
source install/setup.bash
ros2 topic list | grep -E "frontier_markers|blocked_frontiers|peer_state|way_point" 
ros2 topic hz /robot_a/cfpa2/frontier_markers --window 20
```
Expected: `/robot_a/cfpa2/frontier_markers` and `/robot_a/cfpa2_peer_coordination/{peer_state,blocked_frontiers}` exist and publish. If `frontier_markers` topic name differs, set `frontier_markers_topic` in Task 3 to the actual single_robot output and rebuild-free relaunch.

- [ ] **Step 2: Confirm peer negotiation is alive**

```bash
ros2 topic echo /robot_b/cfpa2_peer_coordination/inbox/negotiation_request --once
ros2 topic echo /robot_a/cfpa2_peer_coordination/blocked_frontiers --once
```
Expected: at least one negotiation message and a (possibly empty) blocked_frontiers PoseArray within ~30 s of exploration.

- [ ] **Step 3: Confirm robots receive goals and move**

```bash
ros2 topic hz /robot_a/way_point_coord --window 5
ros2 topic echo /robot_a/odom/nav --field pose.pose.position --once
sleep 20
ros2 topic echo /robot_a/odom/nav --field pose.pose.position --once
```
Expected: way_point publishes; robot_a position changes between the two odom samples (it is moving).

- [ ] **Step 4: Confirm union coverage climbs**

In a second shell, run the metrics logger in union mode against the live decentralised sim for 120 s:
```bash
source install/setup.bash
ros2 run go2w_observability exploration_metrics_logger.py --ros-args \
  -p use_sim_time:=true -p global_coverage_source:=union \
  -p scene_area_m2:=384.0 -p robot_namespaces:="['robot_a','robot_b']" \
  2>&1 | tee /tmp/phase0_metrics.log
```
(If the logger is run via a launch include instead of `ros2 run`, use that path; confirm the executable name with `ros2 pkg executables go2w_observability`.)
Expected: the periodic summary line shows `global_coverage_ratio` strictly increasing over the 120 s window (e.g. 0.0 → >0.15).

- [ ] **Step 5: Tear down**

```bash
./scripts/debug/kill_sim.sh
```

- [ ] **GATE DECISION:** If Steps 3–4 show movement + rising coverage, Phase 0 passes — proceed to Phase 1. If decentralised does NOT explore (no goals, no movement, flat coverage), STOP and report findings (which topic/handshake is broken) before building the dropout matrix. Use `superpowers:systematic-debugging` if blocked.

- [ ] **Step 6: Commit any wiring fixes made during bring-up**

```bash
git add -A && git commit -m "fix(bench): decentralised dual bring-up wiring for demo3_mixed"
```

---

## Phase 1 — Dropout relay + launch wiring

### Task 5: `comms_dropout_relay` node + unit tests for drop logic

**Files:**
- Create: `scripts/runtime/comms_dropout_relay.py`
- Create: `scripts/runtime/dropout_core.py` (pure, ROS-free drop logic)
- Test: `scripts/runtime/test_dropout_core.py`

- [ ] **Step 1: Write the failing test**

```python
# scripts/runtime/test_dropout_core.py
import random
from dropout_core import DropCounter


def test_prob_zero_never_drops():
    dc = DropCounter(drop_prob=0.0, seed=1)
    assert all(not dc.should_drop() for _ in range(1000))
    assert dc.dropped == 0 and dc.passed == 1000


def test_prob_one_always_drops():
    dc = DropCounter(drop_prob=1.0, seed=1)
    assert all(dc.should_drop() for _ in range(1000))
    assert dc.passed == 0 and dc.dropped == 1000


def test_prob_half_is_approximately_half():
    dc = DropCounter(drop_prob=0.5, seed=42)
    n = 20000
    for _ in range(n):
        dc.should_drop()
    frac = dc.dropped / n
    assert 0.47 < frac < 0.53


def test_seed_is_reproducible():
    a = DropCounter(drop_prob=0.5, seed=7)
    b = DropCounter(drop_prob=0.5, seed=7)
    seq_a = [a.should_drop() for _ in range(500)]
    seq_b = [b.should_drop() for _ in range(500)]
    assert seq_a == seq_b
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd scripts/runtime && python -m pytest test_dropout_core.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dropout_core'`

- [ ] **Step 3: Write the pure drop-logic module**

```python
# scripts/runtime/dropout_core.py
"""ROS-free probabilistic drop decision with seeded reproducibility."""
from __future__ import annotations

import random


class DropCounter:
    def __init__(self, drop_prob: float, seed: int = 0) -> None:
        self.drop_prob = max(0.0, min(1.0, float(drop_prob)))
        self._rng = random.Random(seed)
        self.dropped = 0
        self.passed = 0

    def should_drop(self) -> bool:
        drop = self._rng.random() < self.drop_prob
        if drop:
            self.dropped += 1
        else:
            self.passed += 1
        return drop
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd scripts/runtime && python -m pytest test_dropout_core.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Write the ROS 2 relay node**

```python
#!/usr/bin/env python3
# scripts/runtime/comms_dropout_relay.py
"""Topic-interposition relay that drops a configured fraction of messages.

Subscribes ``in_topic``, drops each message with probability ``drop_prob``
(seeded RNG for reproducibility), republishes survivors to ``out_topic``.
One relay process per dropped directed link. Benchmark scaffolding only.
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dropout_core import DropCounter


def _qos(profile: str) -> QoSProfile:
    if profile == "best_effort":
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
    if profile == "transient_local":
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


class CommsDropoutRelay(Node):
    def __init__(self) -> None:
        super().__init__("comms_dropout_relay")
        self.declare_parameter("in_topic", "")
        self.declare_parameter("out_topic", "")
        self.declare_parameter("msg_type", "")  # e.g. nav_msgs/msg/Odometry
        self.declare_parameter("drop_prob", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("qos", "reliable")  # reliable|best_effort|transient_local

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        msg_type = self.get_parameter("msg_type").value
        drop_prob = float(self.get_parameter("drop_prob").value)
        seed = int(self.get_parameter("seed").value)
        qos = _qos(str(self.get_parameter("qos").value))

        if not (in_topic and out_topic and msg_type):
            raise RuntimeError("in_topic, out_topic, msg_type are required")

        self._counter = DropCounter(drop_prob=drop_prob, seed=seed)
        msg_cls = get_message(msg_type)
        self._pub = self.create_publisher(msg_cls, out_topic, qos)
        self._sub = self.create_subscription(msg_cls, in_topic, self._cb, qos)
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"dropout relay {in_topic} -> {out_topic} ({msg_type}) "
            f"p={drop_prob} seed={seed} qos={self.get_parameter('qos').value}")

    def _cb(self, msg) -> None:
        if not self._counter.should_drop():
            self._pub.publish(msg)

    def _log_stats(self) -> None:
        total = self._counter.dropped + self._counter.passed
        frac = (self._counter.dropped / total) if total else 0.0
        self.get_logger().info(
            f"dropped={self._counter.dropped} passed={self._counter.passed} "
            f"frac={frac:.3f}")


def main() -> None:
    rclpy.init()
    node = CommsDropoutRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Make it executable + commit**

```bash
chmod +x scripts/runtime/comms_dropout_relay.py
git add scripts/runtime/comms_dropout_relay.py scripts/runtime/dropout_core.py scripts/runtime/test_dropout_core.py
git commit -m "feat(bench): comms_dropout_relay node + seeded drop-logic tests"
```

---

### Task 6: Wire dropout relays into the launch (`comms_dropout` + `dropout_seed`)

**Files:**
- Modify: `src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py`

**Interposition strategy:** remap the active mode's coordination endpoints to a `__predrop` topic, run one relay per directed link from `__predrop` → the real topic. 6 relays per mode.

- [ ] **Step 1: Declare the args**

```python
    DeclareLaunchArgument("comms_dropout", default_value="0.0",
        description="per-message drop probability on coordination links [0..0.8]"),
    DeclareLaunchArgument("dropout_seed", default_value="0",
        description="seed for the dropout RNG (set per trial)"),
```

Read them:
```python
    comms_dropout = float(LaunchConfiguration("comms_dropout").perform(context))
    dropout_seed = int(LaunchConfiguration("dropout_seed").perform(context))
```

- [ ] **Step 2: Add a relay-factory helper near the top of the setup function**

```python
    relay_runtime = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            get_package_share_directory("go2_gazebo_sim"))))),
        "scripts", "runtime", "comms_dropout_relay.py")
    # Fallback to repo-relative path if share dir is not under the workspace root:
    if not os.path.exists(relay_runtime):
        relay_runtime = os.path.abspath(os.path.join(
            os.getcwd(), "scripts", "runtime", "comms_dropout_relay.py"))

    _relay_idx = [0]
    def make_relay(in_topic, out_topic, msg_type, qos, seed_offset):
        _relay_idx[0] += 1
        return ExecuteProcess(
            cmd=["python3", relay_runtime, "--ros-args",
                 "-p", f"in_topic:={in_topic}",
                 "-p", f"out_topic:={out_topic}",
                 "-p", f"msg_type:={msg_type}",
                 "-p", f"drop_prob:={comms_dropout}",
                 "-p", f"seed:={dropout_seed + seed_offset}",
                 "-p", f"qos:={qos}",
                 "-p", "use_sim_time:=true"],
            output="screen")
```

(Ensure `ExecuteProcess` is imported; it is commonly already imported in this file.)

- [ ] **Step 3: Decentralised relays — remap peer_coordinator outputs to `__predrop`, relay to real**

In the decentralised branch (Task 3), when `comms_dropout > 0`, add remappings to each `peer_coordinator` Node so its outputs publish to `__predrop` topics, then append the relays. For each `ns` with peer `peer`:

```python
        if comms_dropout > 0.0:
            ps_real = f"/{ns}/cfpa2_peer_coordination/peer_state"
            req_real = f"/{peer}/cfpa2_peer_coordination/inbox/negotiation_request"
            resp_real = f"/{peer}/cfpa2_peer_coordination/inbox/negotiation_response"
            # remap on THIS ns's peer_coordinator Node (add to its remappings=[...]):
            #   (ps_real, ps_real + "__predrop"),
            #   (req_real, req_real + "__predrop"),
            #   (resp_real, resp_real + "__predrop"),
            nodes.append(make_relay(ps_real + "__predrop", ps_real,
                "cfpa2_peer_coordination_msgs/msg/PeerState", "best_effort", 1))
            nodes.append(make_relay(req_real + "__predrop", req_real,
                "cfpa2_peer_coordination_msgs/msg/NegotiationRequest", "reliable", 2))
            nodes.append(make_relay(resp_real + "__predrop", resp_real,
                "cfpa2_peer_coordination_msgs/msg/NegotiationResponse", "reliable", 3))
```

Add the three remappings to the corresponding `peer_coordinator` Node's `remappings=` list (create the list if absent). Use distinct `seed_offset` per ns by adding `+10` for the second namespace so a→b and b→a links don't share a seed (e.g. pass `1,2,3` for robot_a and `11,12,13` for robot_b).

- [ ] **Step 4: Centralised relays — remap the coordinator's inputs/outputs to `__predrop`, relay to real**

In the centralised branch, when `comms_dropout > 0`, add remappings to the single `cfpa2_coordinator` Node and append relays, for each `ns` in `['robot_a','robot_b']`:

```python
        if comms_dropout > 0.0:
            map_real = f"/{ns}/map"
            odom_real = f"/{ns}/odom/nav"
            goal_real = f"/{ns}/way_point_coord"
            # remap on the coordinator Node (add to remappings=[...]):
            #   (map_real,  map_real  + "__predrop"),   # starve coordinator's map input
            #   (odom_real, odom_real + "__predrop"),   # starve coordinator's odom input
            #   (goal_real, goal_real + "__predrop"),   # delay coordinator's goal output
            # input relays: real publisher -> coordinator's __predrop subscription
            nodes.append(make_relay(map_real, map_real + "__predrop",
                "nav_msgs/msg/OccupancyGrid", "transient_local", 4))
            nodes.append(make_relay(odom_real, odom_real + "__predrop",
                "nav_msgs/msg/Odometry", "reliable", 5))
            # output relay: coordinator's __predrop -> real goal topic the bridge reads
            nodes.append(make_relay(goal_real + "__predrop", goal_real,
                "geometry_msgs/msg/PointStamped", "reliable", 6))
```

Add the three remappings to the `cfpa2_coordinator` Node's `remappings=` list. Use per-ns seed offsets (`4,5,6` and `14,15,16`).

- [ ] **Step 5: Syntax-check + dry launch with show-args**

```bash
micromamba run -n cmu_env python -c "import ast; ast.parse(open('src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py').read()); print('OK')"
source install/setup.bash
ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py --show-args | grep -E "coordination_mode|comms_dropout|dropout_seed"
```
Expected: `OK`; the three new args appear in `--show-args`.

- [ ] **Step 6: Commit**

```bash
git add src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py
git commit -m "feat(bench): comms_dropout relays wired for both coordination modes"
```

---

### Task 7: Dry-run validation of dropout (both modes, one nonzero level)

**Files:** none; live verification.

- [ ] **Step 1: Decentralised @ 50% — relays drop ≈ half**

```bash
timeout 120 ./scripts/launch/nav_test_demo3_mixed.sh gui:=false rviz:=false \
  coordination_mode:=decentralised cfpa2_executable_suffix:=_cpp \
  comms_dropout:=0.5 dropout_seed:=1 2>&1 | tee /tmp/dryrun_dec50.log &
sleep 90
grep -E "dropout relay|frac=" /tmp/dryrun_dec50.log | tail -20
./scripts/debug/kill_sim.sh
```
Expected: relay log lines for peer_state/negotiation links with `frac` ≈ 0.45–0.55.

- [ ] **Step 2: Centralised @ 50% — relays drop ≈ half + robots still move**

```bash
timeout 120 ./scripts/launch/nav_test_demo3_mixed.sh gui:=false rviz:=false \
  coordination_mode:=centralised comms_dropout:=0.5 dropout_seed:=1 \
  2>&1 | tee /tmp/dryrun_cen50.log &
sleep 90
grep -E "frac=" /tmp/dryrun_cen50.log | tail -10
source install/setup.bash; ros2 topic hz /robot_a/way_point_coord --window 5
./scripts/debug/kill_sim.sh
```
Expected: relay `frac` ≈ 0.5 for map/odom/goal links; way_point still publishes (at a reduced rate). Robots still explore (degraded).

- [ ] **Step 3: Commit any fixes**

```bash
git add -A && git commit -m "fix(bench): dropout dry-run corrections" --allow-empty
```

---

## Phase 2 — Driver + reporting + full run

### Task 8: Per-condition summarizer (table + plots)

**Files:**
- Create: `scripts/bench/summarize_dropout_benchmark.py`
- Test: `scripts/bench/test_summarize_dropout_benchmark.py`

**Metric definitions (reused from the existing harness):** final `global_coverage_ratio` (last CSV row), and time-to-coverage-X% = first sim-time at which `global_coverage_ratio >= X` (mirrors [aggregate_exploration_benchmark.py:239](../../../scripts/bench/aggregate_exploration_benchmark.py#L239)).

- [ ] **Step 1: Write the failing test**

```python
# scripts/bench/test_summarize_dropout_benchmark.py
from summarize_dropout_benchmark import time_to_coverage, final_coverage


def test_final_coverage_takes_last_row():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"},
            {"t_sim": "10", "global_coverage_ratio": "0.4"},
            {"t_sim": "20", "global_coverage_ratio": "0.55"}]
    assert final_coverage(rows) == 0.55


def test_time_to_coverage_returns_first_crossing():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"},
            {"t_sim": "10", "global_coverage_ratio": "0.4"},
            {"t_sim": "20", "global_coverage_ratio": "0.55"}]
    assert time_to_coverage(rows, 0.5) == 20.0


def test_time_to_coverage_none_if_never_reached():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"}]
    assert time_to_coverage(rows, 0.9) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd scripts/bench && python -m pytest test_summarize_dropout_benchmark.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the summarizer**

```python
#!/usr/bin/env python3
# scripts/bench/summarize_dropout_benchmark.py
"""Aggregate dropout-benchmark trial CSVs into a per-condition table + plots.

Layout expected:  OUT_DIR/<mode>/d<pct>/trial_<n>/metrics.csv
Each metrics.csv has at least columns: t_sim, global_coverage_ratio.
"""
from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def _rows(csv_path: Path) -> list[dict]:
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def final_coverage(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(rows[-1]["global_coverage_ratio"])


def time_to_coverage(rows: list[dict], threshold: float):
    for r in rows:
        if float(r["global_coverage_ratio"]) >= threshold:
            return float(r["t_sim"])
    return None


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def _std(xs):
    xs = [x for x in xs if x is not None]
    return statistics.pstdev(xs) if len(xs) > 1 else 0.0


def summarize(out_dir: Path, thresholds=(0.5, 0.7, 0.9)):
    summary = []
    for mode_dir in sorted(p for p in out_dir.iterdir() if p.is_dir()):
        for drop_dir in sorted(p for p in mode_dir.iterdir() if p.is_dir()):
            trials = sorted(drop_dir.glob("trial_*/metrics.csv"))
            finals, ttc = [], {t: [] for t in thresholds}
            for csv_path in trials:
                rows = _rows(csv_path)
                finals.append(final_coverage(rows))
                for t in thresholds:
                    ttc[t].append(time_to_coverage(rows, t))
            entry = {
                "mode": mode_dir.name,
                "dropout": drop_dir.name,
                "n_trials": len(trials),
                "final_cov_mean": _mean(finals),
                "final_cov_std": _std(finals),
            }
            for t in thresholds:
                pct = int(t * 100)
                entry[f"ttc_{pct}_mean"] = _mean(ttc[t])
                reached = [x for x in ttc[t] if x is not None]
                entry[f"ttc_{pct}_reach_rate"] = (
                    len(reached) / len(trials)) if trials else 0.0
            summary.append(entry)
    return summary


def _plot(summary, out_dir: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipping plots")
        return
    modes = sorted({e["mode"] for e in summary})
    fig, ax = plt.subplots()
    for mode in modes:
        pts = sorted((float(e["dropout"].lstrip("d")) / 100.0
                      if e["dropout"].startswith("d") else float(e["dropout"]),
                      e["final_cov_mean"] or 0.0)
                     for e in summary if e["mode"] == mode)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, marker="o", label=mode)
    ax.set_xlabel("comms dropout")
    ax.set_ylabel("final coverage ratio (mean)")
    ax.set_title("Exploration coverage vs comms dropout")
    ax.legend()
    fig.savefig(out_dir / "final_coverage_vs_dropout.png", dpi=120)
    print(f"wrote {out_dir/'final_coverage_vs_dropout.png'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    args = ap.parse_args()
    summary = summarize(args.out_dir)
    cols = list(summary[0].keys()) if summary else []
    print("\t".join(cols))
    for e in summary:
        print("\t".join(str(e[c]) for c in cols))
    with open(args.out_dir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(summary)
    _plot(summary, args.out_dir)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd scripts/bench && python -m pytest test_summarize_dropout_benchmark.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
chmod +x scripts/bench/summarize_dropout_benchmark.py
git add scripts/bench/summarize_dropout_benchmark.py scripts/bench/test_summarize_dropout_benchmark.py
git commit -m "feat(bench): per-condition dropout summarizer + plot"
```

---

### Task 9: Benchmark driver script

**Files:**
- Create: `scripts/bench/benchmark_decentralisation_dropout.sh`

- [ ] **Step 1: Write the driver**

```bash
#!/usr/bin/env bash
# Sweep centralised vs decentralised CFPA2 under comms dropout on demo3_mixed.
# Matrix: MODES x DROPOUTS x NUM_TRIALS, each DURATION_SEC sim-seconds.
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_DIR"

MODES="${MODES:-centralised decentralised}"
DROPOUTS="${DROPOUTS:-0.0 0.3 0.5 0.8}"
NUM_TRIALS="${NUM_TRIALS:-10}"
DURATION_SEC="${DURATION_SEC:-600}"
SCENE_AREA_M2="${SCENE_AREA_M2:-384.0}"
CFPA2_SUFFIX="${CFPA2_SUFFIX:-_cpp}"
OUT_DIR="${OUT_DIR:-/tmp/dropout_bench/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

dpct() { python3 -c "print(f'd{int(float(\"$1\")*100):02d}')"; }

trial=0
for mode in $MODES; do
  for drop in $DROPOUTS; do
    dtag="$(dpct "$drop")"
    for n in $(seq 1 "$NUM_TRIALS"); do
      trial=$((trial+1))
      tdir="$OUT_DIR/$mode/$dtag/trial_$n"
      mkdir -p "$tdir"
      echo "=== [$trial] mode=$mode drop=$drop trial=$n -> $tdir ==="
      ./scripts/debug/kill_sim.sh >/dev/null 2>&1 || true
      sleep 3

      # Launch sim (headless). dropout_seed unique per (mode,drop,trial) for reproducibility.
      seed=$(( (RANDOM % 100000) + n ))
      timeout $((DURATION_SEC + 120)) \
        ./scripts/launch/nav_test_demo3_mixed.sh gui:=false rviz:=false \
          coordination_mode:="$mode" cfpa2_executable_suffix:="$CFPA2_SUFFIX" \
          comms_dropout:="$drop" dropout_seed:="$seed" \
          > "$tdir/launch.log" 2>&1 &
      launch_pid=$!

      # Wait for stack readiness (way_point publisher appears), max 90s.
      for _ in $(seq 1 90); do
        if ros2 topic list 2>/dev/null | grep -q "/robot_a/way_point_coord"; then break; fi
        sleep 1
      done

      # Run the metrics logger in union mode for DURATION_SEC, CSV -> trial dir.
      timeout "$DURATION_SEC" ros2 run go2w_observability exploration_metrics_logger.py \
        --ros-args -p use_sim_time:=true -p global_coverage_source:=union \
        -p scene_area_m2:="$SCENE_AREA_M2" \
        -p robot_namespaces:="['robot_a','robot_b']" \
        -p csv_path:="$tdir/metrics.csv" \
        > "$tdir/metrics.log" 2>&1 || true

      ./scripts/debug/kill_sim.sh >/dev/null 2>&1 || true
      kill "$launch_pid" 2>/dev/null || true
      sleep 3
      echo "seed=$seed" > "$tdir/seed.txt"
    done
  done
done

echo "=== aggregating ==="
python3 scripts/bench/summarize_dropout_benchmark.py "$OUT_DIR" | tee "$OUT_DIR/summary.txt"
echo "Done. Results in $OUT_DIR"
```

- [ ] **Step 2: Confirm the logger executable name + its `csv_path` / `robot_namespaces` params**

```bash
source install/setup.bash
ros2 pkg executables go2w_observability | grep exploration_metrics
grep -nE "declare_parameter\(\"(csv_path|robot_namespaces|scene_area_m2)\"" \
  src/go2w/go2w_observability/scripts/exploration_metrics_logger.py
```
Expected: an `exploration_metrics_logger.py` executable; `csv_path`, `robot_namespaces`, `scene_area_m2` params exist. If a param name differs, fix the driver's `--ros-args` to match before running.

- [ ] **Step 3: Make executable + commit**

```bash
chmod +x scripts/bench/benchmark_decentralisation_dropout.sh
git add scripts/bench/benchmark_decentralisation_dropout.sh
git commit -m "feat(bench): centralised-vs-decentralised dropout benchmark driver"
```

---

### Task 10: Smoke the full pipeline (1 trial × 2 modes × 2 levels, short)

**Files:** none; validation.

- [ ] **Step 1: Run a tiny matrix end-to-end**

```bash
MODES="centralised decentralised" DROPOUTS="0.0 0.5" NUM_TRIALS=1 \
  DURATION_SEC=180 OUT_DIR=/tmp/dropout_smoke \
  ./scripts/bench/benchmark_decentralisation_dropout.sh 2>&1 | tee /tmp/dropout_smoke.log
```
Expected: 4 trial dirs each with a non-empty `metrics.csv`; `final_coverage_vs_dropout.png` + `summary.csv` written; `summary.txt` shows nonzero `final_cov_mean` for the 0.0 conditions.

- [ ] **Step 2: Sanity-check the smoke result**

```bash
column -t -s, /tmp/dropout_smoke/summary.csv
```
Expected: 4 rows (2 modes × 2 dropouts), `n_trials=1`, plausible coverage values (0.0-condition ≥ 0.5-condition within each mode is the expected trend but not guaranteed at n=1).

- [ ] **Step 3: Commit any driver/summarizer fixes**

```bash
git add -A && git commit -m "fix(bench): dropout pipeline smoke corrections" --allow-empty
```

---

### Task 11: Full publishable run + writeup

**Files:**
- Create: `docs/claude/decentralisation_dropout_benchmark_results.md`

- [ ] **Step 1: Launch the full 80-trial matrix (run in background; ~overnight)**

```bash
OUT_DIR=/tmp/dropout_bench/full_$(date +%Y%m%d) \
  ./scripts/bench/benchmark_decentralisation_dropout.sh 2>&1 \
  | tee /tmp/dropout_full.log
```
Expected: completes 80 trials; `summary.csv` + plot produced.

- [ ] **Step 2: Write the results doc**

Capture: the summary table (final coverage mean±std + time-to-{50,70,90}% + reach-rate per condition), the `final_coverage_vs_dropout.png`, and a 1-paragraph interpretation of which architecture degrades more gracefully. Note seeds + scene + trial count for reproducibility.

- [ ] **Step 3: Commit results**

```bash
cp /tmp/dropout_bench/full_*/final_coverage_vs_dropout.png docs/claude/
git add docs/claude/decentralisation_dropout_benchmark_results.md docs/claude/final_coverage_vs_dropout.png
git commit -m "docs(bench): centralised vs decentralised CFPA2 dropout benchmark results"
```

---

## Self-review notes

- **Spec coverage:** Component 1 → Tasks 1-2; Component 2 → Task 5; Component 3 → Tasks 3,6; Component 4 → Tasks 8-9; Phase 0 gate → Task 4; Phase 1 dry-run → Task 7; Phase 2 full run → Tasks 10-11. Union metric used for both modes (Tasks 2,9). All spec sections covered.
- **Known live-only unknowns (resolved by explicit verification steps, not placeholders):** exact single_robot frontier-marker output topic (Task 4 Step 1), logger executable name + param names (Task 9 Step 2). Each has a concrete command to confirm and a fallback action.
- **Risk:** Task 4 is the gate — decentralised has never explored end-to-end. If it fails, stop and report before Phase 1/2.
