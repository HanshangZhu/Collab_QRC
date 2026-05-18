# VLM Standalone Exploration — Onboarding Guide

Single-robot MuJoCo exploration stack where the **goal source** is a Vision-Language Model (VLM) instead of a traditional frontier allocator (CFPA2). Everything below the goal-source layer — A* planner, pure-pursuit controller, CHAMP gait, hybrid wheel/leg mux, stuck recovery — is shared with the traditional baselines and unchanged.

Entry point: `standalone/main_vlm.py`
Comparable baselines: `standalone/main.py` (CFPA2 + simple controller), `standalone/main_champ.py` (CFPA2 + CHAMP gait).

---

## 1. What this does

Every ~12 s, the explorer:

1. Snapshots the live occupancy grid + robot pose.
2. Extracts reachable frontier candidates (BFS through free+unknown, inflation-aware, then A*-validated against the runtime planner).
3. Renders a labeled PNG (gray=unknown, white=free, black=walls, red dot+tick=robot, yellow=frontiers, green X=goal).
4. Ships `{PNG, scene-JSON, mission}` to a remote VLM over HTTP (default Google AI Studio → Gemma 4).
5. Parses the response JSON `{goal:{x,y}, reason, artifact_seen, artifact_pos, done}`.
6. Publishes the goal on `/<ns>/goal_pose` — same topic the A* planner already listens to.
7. Logs the whole round-trip (prompt, image, raw response, decision) to `/tmp/standalone_vlm/session_<ts>/cycle_NNNN_*.{json,txt,png}`.

The HTTP call runs in a background thread so the 500 Hz physics loop is never blocked.

**Mission default**: "Explore the environment and locate any visible colored markers, boxes, or objects of interest. Report their world (x, y) when spotted." Override with `--mission "..."`.

---

## 2. Run it

```bash
# Default: Google AI Studio + Gemma 4 (set GEMINI_API_KEY first)
mjpython standalone/main_vlm.py

# Override provider/model
mjpython standalone/main_vlm.py --provider google --model gemini-2.5-flash
mjpython standalone/main_vlm.py --provider xai --model grok-4-1-fast-non-reasoning

# Offline (no API key) — deterministic mock picks highest-info frontier
mjpython standalone/main_vlm.py --provider mock

# Tuning knobs
mjpython standalone/main_vlm.py --cycle-sec 8.0 --timeout-sec 20.0
mjpython standalone/main_vlm.py --mission "find any blue boxes"

# Headless
mjpython standalone/main_vlm.py --headless --no-map
```

**API keys** load from env or any of: `.env`, `.env.xai`, `.env.openai`, `.env.anthropic`, `.env.google`, `.env.gemini`, `.env.local` at repo root. Format: `KEY=value` one per line.

**Required for default Gemma path**: `GEMINI_API_KEY=...` in `.env.gemini` (or `.env`).

**Use `mjpython`, not `python`** — MuJoCo viewer requires its custom interpreter on macOS.

**List available Google models**:
```bash
python -m standalone.vlm.backend --list-models --provider google
```

---

## 3. File map

```
standalone/
├── main_vlm.py                  # entry point; mirrors main_champ.py
├── config.yaml                  # shared sim/planner/CHAMP/watchdog params
│
├── vlm/
│   ├── explorer.py              # VLMExplorer class — goal source
│   ├── backend.py               # urllib HTTP client (xai/openai/anthropic/google/mock)
│   ├── prompts.py               # system + user prompt builders, JSON parser
│   └── renderer.py              # OccupancyMapper → annotated PNG bytes
│
├── nav/
│   ├── planner.py               # AStarPlanner (4-cell inflation = 0.40 m)
│   └── controller.py            # PurePursuitController
│
├── sim/
│   ├── mujoco_env.py            # MuJoCo wrapper
│   ├── champ_controller.py      # CHAMP gait + IK + leg PD + wheel velocity
│   └── sensor/lidar.py          # ray-cast Mid-360 sim
│
├── mapping/occupancy_grid.py    # OccupancyMapper (log-odds)
├── control/
│   ├── hybrid_cmd_router.py     # wheel/legged auto-switch @ vx=0.18 m/s
│   └── stuck_watchdog.py        # 10 s no-motion → backup recovery
├── observability/
│   ├── metrics_logger.py        # ExplorationMetricsLogger (events + stop trigger)
│   └── map_writer.py            # memmap publisher for live viewer
└── viz/map_viewer.py            # matplotlib subprocess reading the memmap
```

---

## 4. Architecture — data flow

```
                                ┌──────────────────────────────────────────┐
                                │  MuJoCo physics @ 500 Hz                 │
                                │  (env.step)                              │
                                └────────────┬─────────────────────────────┘
                                             │
              ┌──────────────────────────────┼──────────────────────────┐
              ▼                              ▼                          ▼
       Lidar ray-cast @ 5 Hz       Get pose2d → /robot/odom/nav    leg_ctrl.step
       → OccupancyMapper.update    → TF                            (CHAMP gait + wheels)
       → mapper.to_occupancy_grid()
                │                            ▲                          ▲
                │ /robot/map                 │                          │
                ▼                            │ /robot/cmd_vel (backup)  │ vx, wz, wheel_mode
         _current_grid[0]                    │                          │
                │                            │                          │
        ┌───────┴────────┐                   │                  router.tick()
        │                │                   │                          ▲
        │           _maybe_replan()          │                  PurePursuit.compute()
        │           per lidar tick           │                          ▲
        │                │                   │                  controller.set_path()
        │                │                   │                          ▲
        │                ▼                   │                       planner.plan()
        │           planner.plan()           │                          ▲
        │                │                   │                  _plan_and_apply()
        │                ▼                   │                          ▲
        │           controller.set_path()    │                  on /robot/goal_pose
        │                                    │                          ▲
        ▼                                    │                          │
   VLMExplorer.tick(t)  ─── every 12 s ─→  query VLM in bg thread       │
        │                                                               │
        │  on result → BUS.publish("/robot/goal_pose", PoseStamped)  ───┘
        │
        └─── on /robot/nav_status state=unreachable → blacklist goal, skip next time
```

`BUS` is the in-process pub/sub bus from `standalone/core/bus.py` — drop-in replacement for ROS topics, no rclpy/DDS.

---

## 5. Per-VLM-cycle breakdown (`VLMExplorer.tick`)

1. **`_ingest_result()`** — if a background worker finished since last tick, parse its decision, log artifact (if any), publish goal on `/<ns>/goal_pose`. Skip if goal is within `min_replan_dist_m` (0.3 m) of `_last_goal`.

2. **Cycle gate** — if worker still running or `sim_time - _last_query_t < cycle_period_sec` (default 12 s), return.

3. **`_build_scene()`**:
   - Builds int8 grid from mapper (−1 unknown, 0 free, 100 occupied).
   - `_extract_frontiers()` — BFS boundary cells (free with ≥1 unknown 4-neighbor), cluster, drop unreachable centroids using `_reachability_distances()` (4-conn BFS through free+unknown cells with **same inflation** as A*). Rank by info-gain (unknown count in sensor-range disk) minus 0.1 × path-length.
   - `_filter_unreachable()` — drop any candidate within 0.5 m of an entry in `_failed_goals` (recent unreachable history, max 8 entries).
   - **A*-validate every candidate** — `planner.plan(ros_grid, robot_xy, cand)` per candidate; keep first N that produce a path. Eliminates BFS-vs-A* semantic mismatch.
   - **Fallback**: if BFS finds candidates but A* rejects all (robot wedged), surface raw BFS list to VLM with a warning printed.
   - Returns `scene` dict + `frontiers` list. Scene includes `map`, `robot`, `coverage`, `frontier_candidates`, `last_goal`, `artifacts_logged`, `recent_failed_goals`.

4. **`MapRenderer.render_b64()`** — matplotlib Agg backend (mjpython-safe) draws grid + trail + frontier dots + robot dot+heading + green X goal, returns base64 PNG string.

5. **Build prompts** — `build_system_prompt(mission)` + `build_user_prompt(scene)`. Prompt demands strict JSON output with no `<thought>` blocks. See [§6 Prompt contract](#6-prompt-contract).

6. **Persist cycle artifacts** — `/tmp/standalone_vlm/session_<ts>/`:
   - `cycle_NNNN_scene.json` — scene snapshot
   - `cycle_NNNN_user_prompt.txt` — full user message
   - `cycle_NNNN_map.png` — rendered map image
   - (later) `cycle_NNNN_response.txt` — raw VLM output
   - (later) `cycle_NNNN_decision.json` — parsed decision + duration + error

7. **Dispatch background worker** (`threading.Thread`, daemon):
   - `vlm_backend.query_vlm(...)` — POST to provider's `/chat/completions` (or `/messages` for Anthropic) with image_url block + system + user messages. urllib stdlib only.
   - Default endpoint for Google: `https://generativelanguage.googleapis.com/v1beta/openai/chat/completions` with `Bearer GEMINI_API_KEY`.
   - On completion, parse JSON, stash into `_pending_result` (lock-guarded). Next tick `_ingest_result()` picks it up.

8. **`parse_response(raw)`** — `extract_json_object()` strips `<thought>...</thought>` / `<reasoning>...</reasoning>` blocks (Gemma 4 reasoning), markdown code fences, prose preamble; finds last balanced `{...}`. Falls back to greedy first-`{` to last-`}`. Normalizes to `{goal, reason, artifact_seen, artifact_pos, done}`.

9. **Artifact handling** — if `artifact_seen=true` and `artifact_pos` present, append to `_artifacts`, publish `/<ns>/artifact_seen`, persist `artifacts.json` to session log dir.

---

## 6. Prompt contract

**System prompt** (`standalone/vlm/prompts.py`):
- Establishes mission.
- Describes the PNG channels: gray=unknown, white=free, black=walls, red dot+tick=robot, yellow=frontiers, green X=goal.
- Demands strict JSON output **as the very first character**, no `<thought>` blocks, no markdown fences, no prose.
- Rules:
  1. Goal must be within map extents.
  2. Goal must be in free / unknown-but-traversable space.
  3. Frontier list is pre-filtered to reachable cells — pick verbatim or near one, **don't invent distant unreachable goals**.
  4. Avoid points within 0.5 m of `recent_failed_goals`.
  5. If no good direction, `goal: null, reason: "hold"`.
  6. If frontiers empty AND all reachable mapped, `done: true`.

**User prompt** — JSON dump of the scene dict in a fenced ```json block (truncated to 4000 chars if needed; frontier list trimmed to 6 first). The PNG ships as a separate content block (`image_url` with data: URI for OpenAI/xAI/Google; `image/base64` for Anthropic).

**Response schema**:
```json
{
  "goal":          { "x": <float>, "y": <float> } | null,
  "reason":        "<one short sentence>",
  "artifact_seen": <bool, default false>,
  "artifact_pos":  { "x": <float>, "y": <float> } | null,
  "done":          <bool, default false>
}
```

---

## 7. Goal → motion pipeline (shared with `main_champ.py`)

`main_vlm.py` is essentially `main_champ.py` with the CFPA2 node swapped for `VLMExplorer`. The downstream chain is identical:

1. **`_on_goal(msg)`** subscriber on `/<ns>/goal_pose` — dedup vs `_active_goal` within 0.30 m, increment `_goal_seq`, reset `_fail_count`, call `_plan_and_apply(gx, gy, "new_goal")`.

2. **`_plan_and_apply()`**:
   - `planner.plan(grid, pose, goal)` — A* on the ROS-style OccupancyGrid with 4-cell inflation (0.40 m clearance, matching the original ROS2 `AsyncGridPlanner`).
   - If path → `controller.set_path(path)`, update map writer, log success, return True.
   - If no path → increment `_fail_count`, throttled warning log.
   - **Unreachable signal trigger**: emit `/<ns>/nav_status state=unreachable` and clear `_active_goal` if either:
     - `reason == "new_goal"` and first attempt failed (VLM picked something A* refuses — tell it immediately so next cycle picks differently).
     - `reason == "periodic"` and `_fail_count ≥ 3` (mid-flight map change blocked path).

3. **`_maybe_replan()`** — called every lidar tick (~5 Hz). Checks if `_path_blocked()` (any sampled path cell now reads ≥50 in the grid) OR `REPLAN_PERIOD_SEC` (1.0 s) elapsed; replans accordingly.

4. **`PurePursuitController.compute(pose)`** — returns `(vx, wz)` from current path + lookahead 0.8 m.

5. **`HybridCmdRouter.tick()`** — auto-switches to wheel-drive mode when `|vx| ≥ 0.18 m/s` (the original ROS2 `wheel_linear_threshold`). At cruise: legs hold q_stand, wheels drive. Below: legs trot, wheels freewheel.

6. **`ChampController.set_cmd_vel + step()`** — Raibert heuristic foot targets + 12-point Bezier swing + analytical IK + leg PD (kp=100, kd=1) + wheel velocity actuator.

7. **`env.step()`** — MuJoCo physics tick.

8. **`StuckWatchdog.tick()`** (~2 Hz) — 10 s window with <0.20 m travel + active goal → emit `recovery_event="stuck_detected"`, start 4 s backup at −0.10 m/s, then republish cached goal.
   **Key VLM interop**: watchdog also subscribes to `/<ns>/nav_status` and, on `state=unreachable`, **drops its cached goal AND forces backup recovery immediately** (respecting 8 s cooldown). Prevents the stuck-goal republish loop where watchdog kept resurrecting a dead goal.

---

## 8. Failure modes already handled

| Symptom | Root cause | Fix |
|---|---|---|
| VLM picks a goal A* refuses | BFS reachability (no inflation) said yes; A* (4-cell inflation) said no | (a) BFS uses same inflation; (b) every candidate A*-validated before reaching VLM |
| Same unreachable goal across many cycles | `_on_goal` reset `fail_count=0` each new arrival → `UNREACHABLE_FAIL_LIMIT=3` never tripped → no blacklist | Force unreachable signal on `new_goal` first-fail (keep 3-tolerance only for `periodic` mid-flight failures) |
| Stuck-goal republish loop ~14 s | `StuckWatchdog` cached failed goal as `_latest_goal`, republished after backup | Watchdog drops `_latest_goal` AND forces backup on `nav_status=unreachable` |
| Robot wedged: BFS finds candidates A* rejects all | True wedge near inflated obstacle | Fallback surfaces BFS list to VLM with WARNING; watchdog backup eventually frees robot |
| Gemma 4 `<thought>` blocks consume max_tokens | Reasoning runs before JSON, gets truncated | `max_tokens=1024` + prompt demands "JSON FIRST" + parser strips `<thought>` |
| HTTP 404 `gemma-3-27b-it not found` | Model not exposed via Google OpenAI-compat layer | Default switched to `gemma-4-26b-a4b-it` (verified available); `--list-models` helper added |
| Coverage-stagnation kills run after 30 s | VLM cycle is 12 s; default 30 s window kills run after 2-3 cycles | `coverage_stagnant_window_sec=120` + `consec_no_reachable_threshold=6` in `main_vlm.py` |

---

## 9. Configuration knobs

### CLI flags (`main_vlm.py`)
| Flag | Default | What |
|---|---|---|
| `--provider` | `auto` | `auto/xai/openai/anthropic/google/gemini/mock` (auto picks first env key) |
| `--model` | provider default | Override model id |
| `--mission` | builtin artifact-finding prompt | Replace mission line in system prompt |
| `--cycle-sec` | 12.0 | VLM query period in sim seconds |
| `--timeout-sec` | 25.0 | HTTP timeout per VLM call |
| `--headless` | off | No MuJoCo viewer |
| `--no-map` | off | Disable live map subprocess |
| `--config` | `standalone/config.yaml` | Override sim config |

### `standalone/config.yaml` — relevant sections
```yaml
nav:
  planner_inflation_cells: 4   # 0.40 m clearance, matches original AsyncGridPlanner
  lookahead_m: 0.8
  max_vx: 0.5
  max_wz: 1.2
  goal_tolerance_m: 0.35

champ:
  gait_period: 0.50
  duty_factor: 0.50
  swing_height: 0.04
  stance_depth: 0.01
  body_height: 0.36            # FK-locked; do not lower without porting BodyController
  kp_leg: 100.0
  kd_leg: 1.0
  wheel_mode_speed: 0.18       # matches ROS2 wheel_linear_threshold

watchdog:
  stuck_window_sec: 10.0
  stuck_threshold_m: 0.20
  backup_distance_m: 0.40
  cooldown_sec: 8.0
```

### `VLMExplorer` internal constants (edit `standalone/vlm/explorer.py`)
| Param | Default | What |
|---|---|---|
| `min_replan_dist_m` | 0.3 | Skip publishing if new goal within X m of last |
| `max_frontier_cands` | 10 | Cap on candidates shown to VLM |
| `sensor_range_m` | 3.5 | Info-gain disk radius |
| `temperature` | 0.1 | VLM sampling temperature |
| `max_tokens` | 1024 | Output token budget |
| `_failed_goals_max` | 8 | Recent-failure ring buffer size |
| `_failed_radius_m` | 0.5 | Dedupe radius for failed-goals + VLM avoid zone |

---

## 10. Logs and observability

**Per-session VLM log**: `/tmp/standalone_vlm/session_YYYYMMDD_HHMMSS/`
- `cycle_NNNN_scene.json` — exact scene shipped to VLM
- `cycle_NNNN_user_prompt.txt` — user message text
- `cycle_NNNN_map.png` — rendered image
- `cycle_NNNN_response.txt` — raw VLM output
- `cycle_NNNN_decision.json` — parsed decision + duration + error
- `artifacts.json` — running list of `{cycle, pos, reason, t}`

Override with env var `STANDALONE_VLM_LOG=/some/path`.

**Live map viewer** — `standalone/viz/map_viewer.py` runs as a subprocess and reads from a `MapWriter` memmap (`map_writer.py`) updated each lidar tick. Shows live grid + robot + path + goal at ~10 Hz. Independent of the MuJoCo viewer window.

**stdout log lines**:
```
[VLM] cycle 3 returned in 4.2s goal=(2.34, -1.10) reason='Move toward unmapped corridor' ERROR=None
[VLM] published goal (2.34, -1.10)
[VLM] [new_goal] path to (2.34,-1.10) — 27 wps
...
[VLM] [periodic] no path to (2.34,-1.10) (fails=3)
[VLM] goal (2.34,-1.10) unreachable — clearing
[VLM] artifact logged @ (1.20, 2.80) — 'green marker visible at lower-left of map'
```

**`ExplorationMetricsLogger`** publishes a `/<ns>/exploration_complete` String when either:
- `consec_no_reachable_threshold` ticks of `no_reachable` status, or
- coverage Δ < threshold over `coverage_stagnant_window_sec`.
Main loop sees this via `metrics.tick() → True` and shuts down.

---

## 11. Comparison hooks (VLM vs traditional)

`main.py`, `main_champ.py`, and `main_vlm.py` share **A* planner + pure-pursuit + CHAMP gait + hybrid router + stuck watchdog + occupancy mapper + metrics logger**. Only the goal source differs:

| File | Goal source | Notes |
|---|---|---|
| `main.py` | CFPA2 frontier allocator | Original light controller (no CHAMP gait fixes) |
| `main_champ.py` | CFPA2 frontier allocator | Same as main.py + CHAMP gait, wheel/leg mux, replan |
| `main_vlm.py` | `VLMExplorer` (LLM HTTP) | Same as main_champ.py + VLM goal source |

This makes ablation runs straightforward — flip the entry script, hold everything else constant. Future benchmark harness would loop trials per script and aggregate `session/cycle_*` + metrics logger output.

---

## 12. Adding a new VLM provider

1. Add provider id to `resolve_provider()` precedence list in `standalone/vlm/backend.py`.
2. Add env var name to `resolve_api_key()`.
3. Add default model to `default_model()`.
4. Add a branch in `query_vlm()` building the provider-specific HTTP payload. Most providers accept the OpenAI `/chat/completions` shape with `image_url` content blocks; Anthropic uses `/messages` with `image` content type instead.
5. (Optional) extend `_DOTENV_FILES` if you want a dedicated `.env.<provider>` file.

Stdlib only — no SDK dependency. urllib + json + base64.

---

## 13. Known limitations / future work

- **No camera input** — VLM only sees the rendered occupancy PNG, not a MuJoCo RGB camera frame. Scene has 3 green markers + 2 boxes, but the VLM cannot literally "see" them; it can only guess artifact positions from JSON/text cues. Adding a MuJoCo `mjr_render` capture + second image content block would close this gap.
- **`body_height` FK-locked at 0.36 m** — lowering requires porting CHAMP's `BodyController.poseCommand` so `q_stand` is solved from height via IK (current `_ZERO_STANCE` is hardcoded). See `docs/claude/champ_fixes_plan.md` §B.
- **Reachability BFS doesn't model footprint** — A* validation catches this for goals, but the candidate ranker may still favor a centroid the A* path bends sharply to reach. Switching frontier extraction to operate on the *inflated* grid (same the planner uses) would tighten the loop.
- **No benchmark harness yet** — needs a runner that loops N trials per script, varies seeds, aggregates coverage / artifact-find-count / time-to-complete from session logs.
- **First-stride lurch on gait restart** — `PhaseGenerator` + first-stride suppression port from original CHAMP C++ is pending.

---

## 14. Quick reference — topics

All on the in-process `BUS`, namespace `robot` by default.

| Topic | Direction | Payload | Producer | Consumer |
|---|---|---|---|---|
| `/robot/odom/nav` | pub | `Odometry` (pose 2D) | main loop @ 20 Hz | watchdog, VLMExplorer, planner inputs |
| `/robot/map` | pub | `OccupancyGrid` (int8) | main loop per lidar tick | external observers |
| `/robot/goal_pose` | pub | `PoseStamped` | `VLMExplorer._ingest_result` | `_on_goal` in main loop |
| `/robot/nav_status` | pub | `String` JSON `{state, goal_seq, goal_x, goal_y}` | main loop + `_publish_unreachable` | watchdog, VLMExplorer._on_nav_status |
| `/robot/exploration_status` | pub | `String` (state) | VLMExplorer._publish_status | map_writer (display only) |
| `/robot/artifact_seen` | pub | `String` JSON `{cycle, pos, reason, t}` | VLMExplorer | external observers |
| `/robot/cmd_vel` | pub | `Twist` | watchdog backup recovery | main loop backup-cmd override |
| `/robot/recovery_event` | pub | `String` (`stuck_detected` / `backup_started` / `backup_done`) | watchdog | metrics logger |
| `/robot/exploration_complete` | pub | `String` (reason) | metrics logger | main loop stop trigger |

States published on `/robot/exploration_status`: `init`, `querying`, `executing`, `hold`, `no_frontiers`, `done`.
States on `/robot/nav_status`: `navigating`, `goal_reached`, `unreachable`.
