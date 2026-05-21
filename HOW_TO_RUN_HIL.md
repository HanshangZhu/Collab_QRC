# HOW TO RUN — Orin NX HIL bench (teammate handoff)

**What this is:** a Hardware-in-the-Loop bench where **your laptop simulates the
physical world** (MuJoCo ops2 building + fake Mid-360 + CHAMP locomotion) and the
**real Go2's Orin NX** runs the full ROS 1 Noetic autonomy stack (Point-LIO SLAM +
elevation/traversability + Nav2-port `move_base` with CUDA-MPPI + CFPA2 frontier
exploration). Sensors + cmd_vel + viz cross over a **C++ UDP relay** (the Foxy
ros1_bridge is broken on this Jetson). You watch everything live in RViz2 on the
laptop.

```
  ┌─ LAPTOP (you) ──────────────────┐         ┌─ Orin NX (192.168.123.18) ─────┐
  │ MuJoCo ops2 + Mid-360 sim       │  UDP    │ Point-LIO SLAM                 │
  │ CHAMP (drives robot)            │ ──────▶ │ elevation_mapping + trav CNN   │
  │ /livox/lidar 9001, /livox/imu   │  9001-2 │ move_base (SmacLattice+MPPI)   │
  │ RViz2 (viz)            cmd_vel  │ ◀────── │ CFPA2 exploration              │
  │                       9003-5    │  UDP    │ UDP relay (rx sensors/tx viz)  │
  └─────────────────────────────────┘         └────────────────────────────────┘
```

The full design rationale is in [docs/claude/orin_nx_hil_design.md](docs/claude/orin_nx_hil_design.md).

---

## 0. Prerequisites (one-time)

**On your laptop:**
- Ubuntu 22.04 + ROS 2 Humble at `/opt/ros/humble`
- This repo cloned and **built**: `colcon build --symlink-install` (needs the
  `hil_udp_relay`, `pc2_to_livox`, `go2_gazebo_sim` packages — they're in this repo)
- MuJoCo + the `cmu_env` conda/micromamba env (the launch scripts source it)
- `sshpass` installed: `sudo apt install sshpass`

**The Orin NX is shared hardware** — its catkin workspace
(`/home/unitree/autonomous_exploration_zhu`) is already built and the onboard
scripts are already deployed. You do **not** need to build anything on the NX.
(If you ever change `scripts/real/onboard_autonomy_noetic.sh`, re-deploy it — see
§5.)

---

## 1. THE hardware-specific change you need (built-in NIC)

The only thing that differs for your machine is **the network interface to the
Go2**. I was on a flaky USB-Ethernet dongle; **you have a built-in NIC**, so you
skip all the dongle/driver pain. You only need to put your NIC on the Go2 subnet.

The Go2 robot is fixed at **`192.168.123.18`**. Your laptop must have an IP on
`192.168.123.x` (the standard laptop IP is **`192.168.123.222`**).

### Configure your built-in NIC (pick ONE)

Find your NIC name first:
```bash
ip -o link | grep -vE 'lo:|docker|wlp|tailscale'      # e.g. enp2s0 / eno1
```

**Option A — quick, non-persistent (per session):**
```bash
sudo ip addr add 192.168.123.222/24 dev <YOUR_NIC>
sudo ip link set <YOUR_NIC> up
```

**Option B — persistent via nmcli:**
```bash
sudo nmcli con add type ethernet ifname <YOUR_NIC> con-name go2 \
  ipv4.method manual ipv4.addresses 192.168.123.222/24
sudo nmcli con up go2
```

### Verify the link before anything else
```bash
ping -c3 192.168.123.18                  # must get replies
sshpass -p 123 ssh unitree@192.168.123.18 'echo NX_OK'   # must print NX_OK
```
If both pass, you're done with networking. **Do NOT touch any driver/dongle
steps** — those were specific to my AX88179 USB dongle, irrelevant to your
built-in NIC.

> **If you use a laptop IP other than `192.168.123.222`**, export it so the NX
> knows where to send viz/cmd_vel back:
> ```bash
> export LAPTOP_IP=192.168.123.<your>
> ```
> (Honored by `hil_orin_nx.sh`.) Everything else is IP-driven via env defaults —
> no NIC *name* is hardcoded anywhere in the scripts.

---

## 2. Run it

One command brings up both sides (kills stale procs → laptop sim → NX stack):

```bash
./scripts/launch/hil_orin_nx.sh up
```

This:
1. preflight-kills both sides
2. starts the laptop sim (MuJoCo GUI + sensors + CHAMP + **RViz2**) and waits for
   `/livox/lidar`
3. SSHes to the NX and starts the full compute stack in `hil=true` mode

Give it ~60–90 s (the robot does a 20 s stand-up, then SLAM/trav/nav/CFPA2 boot).

### With tunables (optional)
```bash
./scripts/launch/hil_orin_nx.sh up lidar_range=2.5 max_vel=0.4
```
- `lidar_range` — elevation `max_height_range` in m above the sensor (default 1.7;
  raise for tall ceilings, lower to cut overhead clutter)
- `max_vel` — MPPI `vx_max` m/s (default 0.30; raise in open corridors)
- `max_vel_ang` — MPPI `wz_max` rad/s (default 0.8)

### Other subcommands
```bash
./scripts/launch/hil_orin_nx.sh status    # what's running on each side
./scripts/launch/hil_orin_nx.sh monitor   # NX load (tegrastats) + topic rates
./scripts/launch/hil_orin_nx.sh stop      # tear down BOTH sides
```

### Env knobs
| var | default | meaning |
|---|---|---|
| `LAPTOP_IP` | 192.168.123.222 | your laptop's Go2-net IP (NX sends viz/cmd_vel here) |
| `NX_HOST` | 192.168.123.18 | the Go2's NX |
| `NX_PASS` | 123 | NX `unitree` password |
| `NO_RVIZ=1` | — | laptop sim without RViz2 |
| `NO_EXPLORE=1` | — | NX stack with `explore=false` (manual goals, no CFPA2) |
| `POLYFIT_VARIANT` | handwalls | MuJoCo scene variant |

---

## 3. Verify it's working

After `up`, check the chain end-to-end:

```bash
# On the NX (source ROS1): sensors arriving + stack producing
sshpass -p 123 ssh unitree@192.168.123.18 \
  'source /opt/ros/noetic/setup.bash; for t in /livox/lidar /livox/imu \
   /robot/Odometry /robot/traversability_grid /robot/cmd_vel; do \
   printf "%-30s " $t; timeout 4 rostopic hz $t 2>/dev/null | grep -m1 "average rate" \
   || echo NODATA; done'
```
Expect roughly: `/livox/lidar` ~9 Hz, `/livox/imu` ~120–200 Hz, `/robot/Odometry`
~10 Hz, `/robot/traversability_grid` ~5 Hz, `/robot/cmd_vel` ~20 Hz.

On the **laptop**, RViz2 (`hil_nx.rviz`) shows the trav grid + robot trajectory
(viz is relayed back from the NX over UDP ports 9003–9005).

---

## 4. Troubleshooting

**`/livox/lidar` reaches the NX as `NODATA` while `/livox/imu` is fine** — this is
the one real gotcha. The relay fragments the lidar cloud into 60 KB chunks
([udp_protocol.hpp `kMaxFragPayload=60000`](src/go2w/hil_udp_relay/include/hil_udp_relay/udp_protocol.hpp#L52)),
and each 60 KB UDP datagram is IP-fragmented into ~41 pieces by the 1500-MTU NIC.
On a **clean wired link this is fine** (verified: 60 KB datagrams arrive 10/10).
It only failed for me because my USB dongle was dropping packets. **With your
built-in NIC it should just work.** If it doesn't, the durable fix is to reduce
`kMaxFragPayload` to ~1400 (one atomic datagram per fragment, no IP-frag
amplification) and rebuild `hil_udp_relay` on **both** sides.

**RViz2 empty / no robot pose** — viz TF is reconstructed on the laptop from the
relayed Odometry by [`scripts/runtime/odom_to_tf.py`](scripts/runtime/odom_to_tf.py)
+ a static `map→camera_init`. These start automatically inside the laptop sim
launch. If the map shows but no robot, check `/robot/Odometry` is arriving on the
laptop (`ros2 topic hz /robot/Odometry`).

**`cannot SSH` at startup** — your NIC isn't on `192.168.123.x` or the cable is
out. Re-do §1 verification.

**Duplicate processes / weird behavior after a crashed run** — always
`./scripts/launch/hil_orin_nx.sh stop` before re-`up`. If the NX accumulated
duplicates (multiple Point-LIO / move_base), hard-clean it:
```bash
sshpass -p 123 ssh unitree@192.168.123.18 \
  'for p in cfpa2 move_base trav_filter elevation_mapping pointlio laserMapping \
   hil_relay topic_tools static_transform rosmaster roscore; do pkill -9 -f "$p"; done'
```

---

## 5. If you edit the NX-side launch script

`scripts/real/onboard_autonomy_noetic.sh` runs **on the NX**. The NX already has
the current version. If you change it in the repo, re-deploy:
```bash
scp scripts/real/onboard_autonomy_noetic.sh \
  unitree@192.168.123.18:/home/unitree/autonomous_exploration_zhu/scripts/
sshpass -p 123 ssh unitree@192.168.123.18 \
  'chmod +x /home/unitree/autonomous_exploration_zhu/scripts/onboard_autonomy_noetic.sh'
```
(rsync drops the +x bit, so always `chmod +x` after — known gotcha.)

---

## Files in this handoff

| file | side | role |
|---|---|---|
| `scripts/launch/hil_orin_nx.sh` | laptop | **entry point** — orchestrates both sides |
| `scripts/launch/nav_test_hil_nx_desktop.sh` | laptop | MuJoCo world + sensors + CHAMP + RViz2 |
| `scripts/launch/nx_viz_laptop.sh` | laptop | viz-only RX (for real-robot runs; HIL uses the desktop launch) |
| `scripts/runtime/odom_to_tf.py` | laptop | rebuilds TF from relayed Odometry for RViz2 |
| `src/go2w/go2_gazebo_sim/rviz/hil_nx.rviz` | laptop | RViz2 layout (trav grid + trajectory) |
| `scripts/real/onboard_autonomy_noetic.sh` | **NX** | the full onboard stack (hil=true path) |
| `scripts/real/real_run_monitor.sh` | laptop | REAL-robot monitor (real Mid-360, not HIL) |
| `jetson_ws/src/trav_pipeline_ros1/config/elevation_mapping_go2w.yaml` | NX | elevation/trav tuning |
