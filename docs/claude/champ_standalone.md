# Standalone CHAMP Controller — Design Notes

**Context:** `standalone/sim/champ_controller.py` — drop-in replacement for `LegController` in the standalone MuJoCo exploration stack.

---

## What happened (2026-05-11 debugging session)

### Spawn flip — fixed

MJCF spawns at body `z = 0.6 m` with all joints at `q = 0` (legs fully straight, feet
~16 cm above the floor). The settle PD immediately applied ~36 Nm backward thigh torque
(error `0 → 0.9 rad`), pitching the nose down before the legs stabilised → flip.

**Fix:** Warm-start in `ChampController.__init__`: pre-set leg `qpos` to `q_stand` and
body `z` to `body_height` before any physics step.

```python
for i, addr in enumerate(env._leg_qpos_addrs):
    env.data.qpos[addr] = self._q_stand[i]
env.data.qpos[2] = self._p.body_height
env.forward()
```

### Trot flip — root cause and fix

After the spawn fix, the robot navigated for ~45 s (3 path assignments logged) before
flipping mid-trot. The FK was verified exact (sub-micron match against `data.xpos`).
The root cause was the **world-frame stance foot freeze**:

| Tick | Stance foot (world frame) | Body advances | foot_b.x in body frame |
|---|---|---|---|
| 0 | `x=4.20` (world) | body at `x=4.00` | `+0.20` (nominal) |
| N | `x=4.20` (frozen) | body at `x=4.40` | `-0.20` (drifted far back) |

As the body moves forward, the frozen world-frame foot drifts backward in the body frame.
Eventually `L = sqrt(p2x² + p2z²)` approaches `L1 + L2` (kinematic limit), the IK clamps
to the limit joint angles, and the PD applies max torques → flip.

**Fix:** Switch to the CHAMP body-frame algorithm (see below). Foot targets are computed
**fresh from `zero_stance` every tick** — no world-frame accumulation, IK always near
nominal.

---

## Previous controller: `LegController`

`standalone/sim/leg_controller.py` — ~50 lines, dead simple:

```
every tick:
  tau = kp × (q_stand − q) − kd × dq   # hold legs at [0, 0.9, -1.8]
  wheel_vels = cmd_to_wheel(vx, wz)      # ALL motion from wheels
```

- Legs are locked to standing pose — dead weight held at `q_stand = [0, 0.9, -1.8]`.
- All locomotion comes from the four wheel velocity actuators.
- Extremely stable (no gait → no tip risk).
- Used by `main.py` (the baseline exploration entry point).

---

## What is CHAMP?

CHAMP (Control of Hybrid Articulated Mechanical Platforms, Juan Miguel Jimeno 2019-2020)
is the C++ locomotion library used on the **real Go2W** in the ROS2 stack. The robot
receives `cmd_vel_legged` (from the hybrid router) and CHAMP produces
`joint_group_position_controller/commands`.

The full C++ source is archived at `archive/ros2_legacy/champ/champ/include/champ/`.

### Algorithm (4-stage pipeline, all in body frame)

```
cmd_vel (vx, vy, wz)
    │
    ▼  [1] Raibert heuristic        leg_controller.h
    │    step_x = (T_stance/2) × vx
    │    step_y = (T_stance/2) × vy
    │    step_theta = (T_stance/2) × wz × center_to_nominal
    │    → step_length, rotation per leg via transformLeg()
    │
    ▼  [2] Phase generator          phase_generator.h
    │    Sawtooth wave → swing_phase_signal [0→1], stance_phase_signal [0→1]
    │    Trot: diagonal pairs (FL+RR) and (FR+RL) alternate
    │
    ▼  [3] Trajectory planner       trajectory_planner.h
    │    Starting from zero_stance (body frame, fresh each tick):
    │    STANCE: foot sweeps backward
    │        x = (L/2) × (1 − 2 × stance_phase)
    │        y = −stance_depth × cos(π × x / L)
    │    SWING: 12-point Bezier arc
    │        x: −L/2 → +L/2  (control_points_x scaled by step_length)
    │        y: 0 → swing_height → 0  (control_points_y scaled by swing_height)
    │
    ▼  [4] IK                       kinematics.h
         foot position (body frame) → (q_hip, q_thigh, q_calf)
```

### Key property: always in body frame

CHAMP's `zero_stance()` is the nominal foot position in the body frame at the target
standing height. Every tick:
1. Body controller maps desired height/pose to foot targets relative to `zero_stance`.
2. Trajectory planner adds the stance/swing delta to that position.
3. IK converts the final body-frame foot position to joint angles.

No world-frame foot positions are stored. No drift. No kinematic singularities from
accumulated error.

### 12-point Bezier swing arc (trajectory_planner.h)

```
Reference control points (x, y):
  x: [-0.15, -0.2805, -0.3, -0.3, -0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15]
  y: [-0.5,  -0.5,  -0.361, -0.361, -0.361, -0.361, -0.361, -0.321, -0.321, -0.321, -0.5, -0.5]

Scaled at runtime:
  cx[i] = ref_x[i] × (step_length / 0.4)   (endpoints forced to ±step_length/2)
  cy[i] = −((ref_y[i] × h_ratio) + 0.5 × h_ratio)  where h_ratio = swing_height / 0.15

Bezier sum (n=11):
  x = Σ C(11,i) × t^i × (1-t)^(11-i) × cx[i]
  y = −Σ C(11,i) × t^i × (1-t)^(11-i) × cy[i]
```

The Bezier produces `(Δx, Δy)` relative to `zero_stance`. At `t=0` (liftoff): `x = −L/2`,
`y = 0`. At `t=1` (touchdown): `x = +L/2`, `y = 0`. Peak height at `t ≈ 0.5`.

---

## Controller comparison

| Dimension | `LegController` | `ChampController` (ours) |
|---|---|---|
| **Locomotion** | Wheels only | Trot gait (legs walk, wheels freewheel) |
| **Foot target** | Fixed joint angles `q_stand` | Body-frame position from `zero_stance` + trajectory |
| **Stance model** | N/A | Backward sweep in body frame (fresh each tick) |
| **Swing model** | N/A | 12-point Bezier arc (matches CHAMP C++) |
| **Raibert** | N/A | `step = (T_stance/2) × v` → `step_length, rotation` per leg |
| **World-frame tracking** | N/A | None — body frame only, no drift possible |
| **IK** | N/A | Analytical 3-DOF, verified vs `data.xpos` |
| **Body height control** | None | Passive: stance foot at `zero_stance[z]` → PD drives extension → ground reaction |
| **Stability** | Very stable | Stable (body-frame approach prevents IK divergence) |
| **Speed** | ~0.3 m/s (wheel limited) | ~0.2 m/s walk target |
| **LOC** | ~50 | ~280 |

---

## Why we can't just copy the C++ CHAMP

The real stack runs CHAMP as a compiled ROS2 node that:
1. Subscribes to `/cmd_vel_legged` (Twist)
2. Publishes `/joint_group_position_controller/commands` (Float64MultiArray, position targets)

In standalone MuJoCo we write torques directly to `data.ctrl` at 500 Hz. There is no
ROS2 bridge, no position controller, no joint_group_position_controller. We must port
the algorithm to Python.

The port in `champ_controller.py` faithfully implements the 4-stage pipeline above,
using the same Bezier control points, the same Raibert formula, and the same body-frame
approach.

---

## Go2W kinematic constants (verified vs MJCF)

```
Hip origins (body frame): FL(+0.1934, +0.0465, 0)  FR(+0.1934, -0.0465, 0)
                           RL(-0.1934, +0.0465, 0)  RR(-0.1934, -0.0465, 0)
Hip lateral offset d:      FL/RL = +0.0955 m,  FR/RR = −0.0955 m
Thigh length L1:           0.213 m
Calf length  L2:           0.2264 m
Wheel radius:              0.086 m

zero_stance (FK at q=[0, 0.9, -1.8]):
  FL: [+0.204, +0.142, -0.273]   FR: [+0.204, -0.142, -0.273]
  RL: [-0.177, +0.142, -0.273]   RR: [-0.177, -0.142, -0.273]

center_to_nominal (XY):   FL/FR ≈ 0.249 m,  RL/RR ≈ 0.227 m
```

FK verified: `_fk_foot(0, 0.9, -1.8, i)` matches `data.xpos[foot_body_i]` to
`< 3 × 10⁻⁶ m` after settle (see 2026-05-11 debug session).

---

## Gait parameters (config.yaml `champ:` block)

| Parameter | Value | Meaning |
|---|---|---|
| `gait_period` | 0.50 s | Full trot cycle |
| `duty_factor` | 0.50 | 50% stance, 50% swing |
| `swing_height` | 0.08 m | Peak foot lift |
| `stance_depth` | 0.00 m | Ground depression during stance (0 = flat) |
| `body_height` | 0.36 m | Target CoM height |
| `kp_leg` | 60.0 | Joint PD proportional gain |
| `kd_leg` | 3.0 | Joint PD derivative gain |
| `wheel_mode_speed` | 1.0 m/s | Speed threshold for wheel-only mode |

`stance_duration = gait_period × duty_factor = 0.25 s`  
`Raibert step at vx=0.2 m/s: step_x = 0.125 × 0.2 = 0.025 m`
