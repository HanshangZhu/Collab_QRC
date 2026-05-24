# Centralised vs Decentralised CFPA2 — Comms-Dropout Exploration Benchmark

**Date:** 2026-05-24
**Branch:** `bench/decentralisation-dropout`
**Status:** Design approved (chat, 2026-05-24)

## Goal

Measure how **exploration speed** degrades with **inter-robot communication dropout**, comparing two CFPA2 coordination architectures:

- **Centralised** — one `cfpa2_coordinator_node` (joint MDVRP allocator over both robots) publishes goals to both.
- **Decentralised** — `springishere`/`springsnow190327`'s peer-to-peer C++ stack: each robot runs its own `cfpa2_single_robot_node` + `peer_coordinator_node`, negotiating frontier claims pairwise (RACER-style).

Run matrix: **2 modes × 4 dropout levels {0%, 30%, 50%, 80%} × 10 trials × 600 sim-seconds** on demo3_mixed (Go2W=robot_a + Go2=robot_b), headless MuJoCo. = 80 trials, ~overnight.

The interesting result is the *shape of the speed-vs-dropout curve per architecture* — the hypothesis (from RACER framing) being that decentralised degrades more gracefully than centralised as the coordination channel deteriorates.

## Scope decisions (settled in brainstorming)

| Decision | Choice | Rationale |
|---|---|---|
| What "dropout" drops | **Coordination link only**, probabilistic per-message | Apples-to-apples "lose the coordination channel". Matches RACER comms-loss framing. |
| Decentralised links dropped | `peer_state`, `negotiation_request`, `negotiation_response` (each direction) | The entire peer-to-peer surface. |
| Centralised links dropped | each robot's `map`+`odom` *into* coordinator, and coordinator's `goal_pose` *out* to each robot, all at same p | Symmetric "coordination channel loss"; the centralised analog of peer-to-peer. |
| Map-share link | **NOT dropped** | `peer_map_merger` (Haichen) is unfinished/un-runnable; out of scope. |
| Scene | demo3_mixed only | Keeps matrix small; validate pipeline before expanding. |
| Robots | 2 (Go2W robot_a + Go2 robot_b) | Decentralised negotiation is pairwise. |
| Run budget | 10 trials × 600 s (publishable, golden-rule-8 ≥10) | Gated behind a dry run so we don't burn ~16 h on a broken pipeline. |
| Coverage metric | **Union of per-robot explored cells / scene_area** | Architecture-agnostic; no dependency on a `/merged_map` topic only the centralised stack publishes. |

## What already exists (reuse)

- **Coverage-over-time harness**: [exploration_metrics_logger.py](../../../src/go2w/go2w_observability/scripts/exploration_metrics_logger.py) logs per-robot trajectory/coverage and a global coverage ratio to CSV; [aggregate_exploration_benchmark.py](../../../scripts/bench/aggregate_exploration_benchmark.py) computes time-to-coverage-X%, area/m efficiency, means/stdev across trials; [benchmark_exploration_planners.sh](../../../scripts/bench/benchmark_exploration_planners.sh) is the existing multi-trial driver pattern.
- **Centralised dual** already runs: [nav_test_demo3_mixed.sh](../../../scripts/launch/nav_test_demo3_mixed.sh) → [nav_test_mujoco_fastlio_mixed.launch.py](../../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py) starts a single `cfpa2_coordinator_node` with `namespaces:[robot_a,robot_b]`.
- **Decentralised C++ nodes are built**: `install/cfpa2_peer_coordination/lib/cfpa2_peer_coordination/peer_coordinator_node` present; `cfpa2_single_robot_node[_cpp]` already consumes `/{ns}/cfpa2_peer_coordination/blocked_frontiers` ([cfpa2_single_robot.cpp:64](../../../src/collaborative_exploration/cfpa2_collaborative_autonomy/src/cfpa2_single_robot.cpp#L64)).

## What's missing (build)

### Component 1 — Architecture-agnostic coverage metric
In [exploration_metrics_logger.py](../../../src/go2w/go2w_observability/scripts/exploration_metrics_logger.py):
- Add param `global_coverage_source: merged_map | union` (default keep current `merged_map` for back-compat; benchmark uses `union`).
- In `union` mode, set `_global_map_known_cells = len(set.union(*[rs.explored_cells ...]))` at log time instead of from the `/merged_map` callback. The union is already computed for overlap% — reuse it.
- `_global_map_resolution` in union mode = the per-robot map resolution (assert both equal; they share a world frame via common-origin Fast-LIO init, the same assumption the existing overlap% calc relies on).
- `scene_area_m2` set to demo3_mixed free area so `global_coverage_ratio` is meaningful (reuse `SCENE_AREA_M2` already in the existing bench script).

**Boundary/interface:** input = each robot's `/{ns}/map` (already subscribed) + GT odom; output = unchanged CSV schema. No new topics. Works identically for centralised and decentralised because it never reads a coordinator-only topic.

### Component 2 — Comms-dropout relay node
New `scripts/runtime/comms_dropout_relay.py` (ROS 2 node):
- Params: `in_topic`, `out_topic`, `msg_type` (fully-qualified, e.g. `cfpa2_peer_coordination_msgs/msg/PeerState`), `drop_prob` (0..1), `seed` (int, for reproducibility), `qos` (best_effort/reliable to match source).
- Behaviour: subscribe `in_topic`, for each message draw `rng.random() < drop_prob` → drop, else republish to `out_topic`. Log dropped/passed counts periodically.
- Dynamic msg type load via `rosidl_runtime_py.utilities.get_message`.
- One relay process per dropped link, configured by the launch.

**Why a relay, not alternatives:** DDS/netem can't target individual topics and isn't reproducible per-topic; editing the C++ peer_coordinator/coordinator is invasive and needs recompiles. The relay is pure scaffolding, topic-level, seedable.

**Topic interposition wiring:**
- *Decentralised:* `peer_coordinator_node` publishes its outputs on a `*_raw` suffix; relay republishes to the canonical topic the peer subscribes to. Done via launch remap of the publisher node's output topics + a relay per (robot × {peer_state, negotiation_request, negotiation_response}).
- *Centralised:* relay sits on each robot's `map`/`odom` feeding the coordinator (coordinator subscribes to a `*_dropped` topic) and on the coordinator's per-robot `goal_pose` output. Confirm exact input topic names the coordinator consumes during Phase 0.

### Component 3 — Launch wiring
Extend the mixed launch (or a thin benchmark wrapper around it) with:
- `coordination_mode:=centralised|decentralised` (default `centralised` = today's behaviour).
  - `centralised`: existing single `cfpa2_coordinator_node`.
  - `decentralised`: 2× `cfpa2_single_robot_node[_cpp]` + 2× `peer_coordinator_node` (peer_namespaces cross-wired a↔b), no central coordinator.
- `comms_dropout:=0.0..0.8` (default 0.0). When >0, insert the Component-2 relays for the active mode's links and apply the remaps.
- `dropout_seed:=<int>` threaded to relays (per-trial seed for reproducibility).

Prefer adding args to the existing launch over a fork, following the existing `cfpa2_executable_suffix` arg pattern. If the mixed launch is too tangled to extend safely, a dedicated `benchmark_decentralisation.launch.py` that composes the shared sim + per-mode coordination layer is the fallback.

### Component 4 — Benchmark driver + reporting
New `scripts/bench/benchmark_decentralisation_dropout.sh`:
- Env-tunable: `MODES` (default `centralised decentralised`), `DROPOUTS` (default `0.0 0.3 0.5 0.8`), `NUM_TRIALS` (default 10), `DURATION_SEC` (default 600), `OUT_DIR`, `SCENE_AREA_M2`.
- For each (mode, dropout, trial): preflight-kill stale sim, launch headless with the right args + per-trial `dropout_seed`, start the logger in `union` mode, run `DURATION_SEC` sim-seconds, tear down, save trial CSV under `OUT_DIR/<mode>/d<pct>/trial_<n>/`.
- After all trials: run `aggregate_exploration_benchmark.py` grouped by `(mode, dropout)`; emit a summary table (final coverage mean±std, time-to-{50,70,90}% mean±std, reach-rate) + plots: coverage-vs-time per condition, and final-coverage / time-to-X% vs dropout per mode.
- Aggregator may need a small extension to group by `(mode, dropout)` keys (currently `(planner, scene)`); add a generic grouping key rather than hardcoding.

## Execution phases (the dominant risk is decentralised bring-up)

The decentralised stack has **never produced a working end-to-end exploration run** — the last decentralised test in [decentralisation.md](../decentralisation.md) was stuck at `Waiting for map topic from: robot_a/robot_b`, proving only blocked-frontier *comms*, not goal-driven exploration. So a large fraction of this work is bring-up/debugging, not benchmark plumbing.

- **Phase 0 (GATE):** Get decentralised dual *exploring at all* at 0% dropout on demo3_mixed — both robots receive goals, move, and union-coverage climbs over a single 600 s run. Build Component 1 (union metric) here too so we can see coverage. **If decentralised won't explore without deep debugging, STOP and report before building the matrix.**
- **Phase 1:** Build Components 2–3 (relay + launch args). Validate with a 1-trial dry run per mode at one nonzero dropout (e.g. 50%): relays show nonzero drop counts, both modes still produce a coverage curve, CSVs well-formed.
- **Phase 2:** Build Component 4 (driver + reporting). Run the full 80-trial matrix. Aggregate, plot, write up.

## Non-goals / constraints

- **No production CFPA2 or peer_coordination C++ is modified.** All new code is benchmark scaffolding: relay node, launch args, driver script, an opt-in logger metric mode, and a generic aggregator grouping key.
- No HIL / real-robot; sim only.
- No map-merge (Haichen's piece); coverage uses the union metric instead.
- Not a "kill one peer mid-run" event demo — that's a separate experiment (the doc's original success criterion). This is the continuous speed-vs-dropout curve the user asked for.
- Reproducibility: per-trial seeds for the dropout RNG; sim/MuJoCo determinism is best-effort (golden rule: ≥10 trials precisely because per-run variance exists).

## Success criteria

1. Decentralised dual explores demo3_mixed end-to-end at 0% dropout (Phase 0 gate).
2. Dropout relays verifiably drop ≈ the configured fraction (logged counts).
3. Full 80-trial matrix completes; aggregated table + plots produced.
4. The same union coverage metric is used for both architectures (fair comparison).
