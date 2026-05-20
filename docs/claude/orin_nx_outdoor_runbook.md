# Orin NX outdoor-autonomy runbook — progressive plan (loop → indoor → outdoor)

Goal for the day: get the Go2 autonomously exploring outdoors (ops). Realistic
path is **progressive** — each stage produces a deliverable; outdoor is the
bonus, not an all-or-nothing bet. Estimated by stage so you can stop anywhere
with something working.

**Prereqs each morning:** robot charged (NX is robot-powered), USB-eth link up
(`cat /sys/class/net/enxc8a36240a4c7/carrier` == 1; reseat dongle if 0 — the
AX88179 flakes; consider a spare adapter). NX is `unitree@192.168.123.18` / `123`.
NX needs internet via laptop NAT for any apt/pip (see .orin_nx_cheatsheet.md).

---

## Stage 1 — Close the cmd_vel loop (HIL, ~1-2h) ★ #1 BLOCKER

Why first: nothing moves without it, and it's pure software (no real robot/outdoor
variables). Use the HIL bench (laptop sim + NX compute).

```bash
# laptop:
./scripts/launch/hil_orin_nx.sh up        # MuJoCo + relay + NX stack (or run halves manually)
# NX (diagnose where the chain breaks):
ssh unitree@192.168.123.18
  ~/autonomous_exploration_zhu/scripts/../../scripts/real/diag_cmdvel_chain.sh   # or copy diag script to NX
```
The diag script walks CFPA2 goal → bridge → move_base → cmd_vel and names the
first broken link. Most likely fixes (from desktop-standalone history in CLAUDE.md):
- **bridge not forwarding**: goal not changing while robot stationary → manual-goal test.
- **move_base EMPTY plan**: start-in-collision / footprint vs inflation. Tune
  `inflation_radius` (ops2 used 0.16) + `consider_footprint:true`.
- **goal in unknown space**: SmacLattice `allow_unknown` / CFPA2 reachability.
- Manual goal sanity: `rostopic pub -1 /robot/move_base_simple/goal geometry_msgs/PoseStamped '{header:{frame_id: map}, pose:{position:{x:1.5}, orientation:{w:1.0}}}'` → cmd_vel should appear.

**Done when:** in HIL, the MuJoCo robot physically drives toward CFPA2 frontiers
(cmd_vel 20Hz, odom bbox grows). THIS proves end-to-end autonomy logic.

## Stage 2 — Real Point-LIO on the real Mid-360 (~1-2h)

Why: HIL fed simulated lidar. Real Mid-360 + walking Go2 is where SLAM breaks
(CLAUDE.md: z-drift +5.8m if IMU init during stand-up). Validate SLAM ALONE
before adding nav.

```bash
# NX, real sensors (NOT hil):
~/autonomous_exploration_zhu/scripts/onboard_autonomy_noetic.sh explore=false   # SLAM+trav+nav, no auto-goals
# verify on the robot, stationary then hand-walked:
rostopic hz /robot/Odometry          # ~10Hz, stable
rostopic echo -n1 /robot/Odometry/pose/pose/position   # z should NOT drift (<0.2m)
```
- **MUST**: lifecycle-gate Point-LIO IMU init until robot settled (stand-up done).
  Do NOT use SIM_GT_ODOM (that's sim-only).
- Watch z-drift: if it climbs, the IMU init fired during motion — fix gating.

**Done when:** carry/walk the robot a few meters, SLAM trajectory matches reality
(~1m/s, no flying z).

## Stage 3 — Indoor / flat autonomous exploration (~2-3h)

Why: controlled space, e-stop reach, before committing to outdoor ops.

```bash
# robot ON THE GROUND, clear area, e-stop ready:
~/autonomous_exploration_zhu/scripts/onboard_autonomy_noetic.sh explore=true
```
- Re-tune for REAL corridor widths (SLAM mesh ≠ real building): `inflation_radius`,
  `footprint`. CLAUDE.md golden rule 14: `consider_footprint:true` + polygon.
- `/mujoco/contacts` collision detection is sim-only — gone on real robot; watch physically.
- stuck_watchdog / recovery behaviors active.

**Done when:** robot autonomously picks frontiers and drives to them indoors,
no human goal input, no collisions.

## Stage 4 — Outdoor ops autonomous exploration (BONUS)

Only if 1-3 are solid. New variables: GPS-denied outdoor SLAM drift, uneven
terrain (trav CNN tuned indoors), sunlight on Mid-360, battery under load.
- Start in a bounded outdoor area, supervised, e-stop in hand.
- Expect trav-grid re-tuning for outdoor ground texture.
- Battery: outdoor walking drains fast — plan a charge buffer.

---

## Hard-won gotchas (don't re-learn these tomorrow)
- **ops2-v4 scene meshes** (`bags/meshes/ops2_cuda/`) are gitignored — must be on
  the laptop for HIL. XML paths are 6-level (correct, resolved via meshdir). Synced.
- **pc2_to_livox** must read `/robot/registered_scan_reliable` (the qos_bridge'd
  RELIABLE topic), not the raw BestEffort lidar topic.
- **ros1_bridge is dead** on the NX (Foxy bad_alloc) — we use the C++ UDP relay.
- **conda poison**: strip miniconda from PATH before ROS (onboard scripts do this).
- **NX no internet** by default — laptop NAT (cheatsheet).
- **link flakes**: detached `setsid` scripts survive SSH drops; short-connection retries.
