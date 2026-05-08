#!/bin/bash
# real_autonomy_se2.sh — STREAMLINED entry: SE2 holonomic, the only navigation
# config we tune for as of 2026-05-05.
#
# Why this exists
# ---------------
# real_autonomy.sh accumulated a profile-selection matrix (off | omni_2d |
# se2_holonomic) that's still useful for ad-hoc comparison runs but bad UX
# for daily ops — too many flags, too many ways to launch the wrong thing.
# This wrapper bakes in:
#
#   holonomic_profile = se2_holonomic   (SmacPlannerLattice + diff primitives,
#                                        forward-bias MPPI, no lateral strafe)
#   nav               = nav2_mppi       (production stack)
#
# and forwards a curated subset of flags that actually matter day-to-day:
#
#   robot       = go2w | go2          default: go2w
#   slam        = auto | carto_l1 |
#                 fastlio_mid360       default: auto
#   oa          = true | false         default: false
#                 (false → /api/sport/request, no manual pre-arm needed)
#   execute     = true | false         default: true
#   manual      = true | false         default: false
#                 (true = RViz click-to-nav, exploration paused)
#   record      = true | false         default: true
#   lidar_range = float (m)            default: 8.0
#   cal_pitch_offset_deg = float       default: 0.0
#                 (Go2/Go2W CHAMP stance: -0.7°)
#
# Anything else (carto_mode, holonomic_profile, nav_backend, mapper, etc.)
# is locked. If you need to override one of those for a one-off, use the
# legacy real_autonomy.sh — it still works.
#
# Examples
# --------
#   ./real_autonomy_se2.sh                                # default Go2W stack
#   ./real_autonomy_se2.sh slam=fastlio_mid360
#   ./real_autonomy_se2.sh lidar_range=4.0 oa=false
#   ./real_autonomy_se2.sh stop                           # kill everything
#
# To revert to the multi-profile launcher: scripts/real/real_autonomy.sh

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Stop subcommand pass-through (so users don't have to remember which entry
# they started from).
if [[ "${1:-}" == "stop" ]]; then
    exec "$SCRIPT_DIR/real_autonomy.sh" stop
fi

# Daily-ops defaults that differ from real_autonomy.sh's broad-spectrum defaults.
# oa=false: sport API direct (api_id=1008), no manual pre-arm gymnastics.
# Everything else inherits real_autonomy.sh defaults; user CLI overrides win.
DEFAULTS=(
    "holonomic_profile=se2_holonomic"
    "nav=nav2_mppi"
    "oa=false"
)

# Forward to real_autonomy.sh: profile/nav baked in, oa default flipped to
# false. User CLI args come AFTER so they can override anything (e.g. to
# pass oa=true if they really want obstacles_avoid mode).
exec "$SCRIPT_DIR/real_autonomy.sh" "${DEFAULTS[@]}" "$@"
