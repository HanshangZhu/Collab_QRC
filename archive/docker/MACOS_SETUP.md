# macOS Setup — Running MuJoCo Sim via Docker

## Why Docker?
ROS 2 Humble requires Ubuntu 22.04. macOS not supported natively.
Docker runs Ubuntu 22.04 container on Mac. XQuartz forwards GUI windows (MuJoCo viewer + RViz2) to Mac screen.

---

## Files Created
| File | Purpose |
|------|---------|
| `docker/sim.Dockerfile` | Ubuntu 22.04 + ROS 2 Humble + all Python/ROS deps |
| `docker/docker-compose.sim.yml` | Volume mounts + X11 display env |
| `docker/run_sim.sh` | One-command launcher (XQuartz + Docker) |

---

## One-Time Setup

### 1. XQuartz (done)
- Installed via `brew install --cask xquartz`
- Checked "Allow connections from network clients" in XQuartz → Preferences → Security
- **Restart Mac** before using (required for X11 socket to appear)

### 2. Docker Desktop
- Download and install: https://www.docker.com/products/docker-desktop/
- Open Docker Desktop app, wait for whale icon in menu bar to stop animating

### 3. Build Docker Image (~10 min)
```bash
cd "/Volumes/Harmish SSD/Collab_QRC"
docker compose -f docker/docker-compose.sim.yml build
```

### 4. Build ROS Workspace inside Docker (~20 min)
Artifacts write to host via volume mount and persist after container exits.
```bash
cd "/Volumes/Harmish SSD/Collab_QRC"
docker run --rm \
  -v "$(pwd)":/workspace \
  collab-qrc-sim \
  bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    mkdir -p src/mtare_ros1_ws &&
    touch src/mtare_ros1_ws/COLCON_IGNORE &&
    colcon build --symlink-install --cmake-clean-cache \
      --cmake-args -DPython3_EXECUTABLE=\$(which python3)
  "
```

---

## Every Session

```bash
cd "/Volumes/Harmish SSD/Collab_QRC"

# Open Docker shell (starts XQuartz + ROS sourced automatically)
./docker/run_sim.sh

# Inside container — launch single robot SE2 sim:
./scripts/launch/nav_test_mujoco_fastlio.sh
```

---

## Current Status (2026-05-06)
- [x] XQuartz installed + configured
- [ ] Mac restart pending (required before GUI works)
- [x] Docker Desktop install + start
- [x] Docker image build (`collab-qrc-sim`)
- [ ] ROS workspace build (`colcon build`) — in progress
- [ ] First sim run

---

## Troubleshooting

| Error | Fix |
|-------|-----|
| `dial unix .../docker.sock: no such file` | Docker Desktop not running — open the app |
| No MuJoCo window appears | XQuartz not running or Mac not restarted after install |
| `install/setup.bash not found` | Workspace not built yet — run Step 4 |
| Build errors in colcon | Check `log/latest_build/` inside container for per-package errors |

## Caveats
- **`MUJOCO_GL=glfw`** (default in `run_sim.sh` / compose) = MuJoCo viewer window through XQuartz. If the window fails to open, try `LIBGL_ALWAYS_INDIRECT=1` in the container.
- **`MUJOCO_GL=osmesa`** = software off-screen only (no interactive viewer). Use for headless: `MUJOCO_GL=osmesa ./docker/run_sim.sh`
- VLM demos need `.env.xai` at repo root with `XAI_API_KEY=...`
- YAML/Python changes: instant (symlink-install). C++ changes: need `colcon build` again.
