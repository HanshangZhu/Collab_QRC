#!/usr/bin/env bash
# run_cfpa2.sh — Step 3: CHAMP+CFPA2 trials.
#
# Uses main_champ.py (full CHAMP trot + CFPA2 single-robot frontier allocator).
# This is the no-ROS2 CHAMP baseline corresponding to Burgard 2005 utility
# (info-gain - travel cost - switching penalty).
#
# Output:
#   logs/cfpa2_champ/
#     ├── reports/    ← cfpa2_report_*.json
#     ├── timeseries/ ← coverage_cfpa2_*.csv
#     └── trial_logs/ ← stdout per trial
#
# Usage:
#   ./scripts/bench/run_cfpa2.sh             # 5 trials default
#   NUM_TRIALS=3 ./scripts/bench/run_cfpa2.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM_TRIALS="${NUM_TRIALS:-5}"
MJPYTHON="${MJPYTHON:-mjpython}"

OUT_ROOT="$REPO_ROOT/logs/cfpa2_champ"
mkdir -p "$OUT_ROOT"/{reports,timeseries,trial_logs}

log() { echo "[cfpa2 $(date +%H:%M:%S)] $*"; }

log "==========================================="
log "Step 3: CHAMP+CFPA2 × $NUM_TRIALS trials"
log "OUT: $OUT_ROOT"
log "==========================================="

for trial in $(seq 1 "$NUM_TRIALS"); do
    ts=$(date +%Y%m%d_%H%M%S)
    trial_log="$OUT_ROOT/trial_logs/trial${trial}_${ts}.log"
    ts_before=$(date +%s)

    log "[trial $trial/$NUM_TRIALS] starting → $trial_log"

    STANDALONE_RESULTS_DIR="$OUT_ROOT/reports" \
    "$MJPYTHON" "$REPO_ROOT/standalone/main_champ.py" \
        --headless --no-map \
        > "$trial_log" 2>&1 || true

    # Move coverage CSV from reports/ to timeseries/
    shopt -s nullglob 2>/dev/null || true
    for f in "$OUT_ROOT/reports"/coverage_*.csv; do
        [ -f "$f" ] || continue
        mv "$f" "$OUT_ROOT/timeseries/" 2>/dev/null || true
    done

    elapsed=$(( $(date +%s) - ts_before ))
    log "[trial $trial] done in ${elapsed}s"
    sleep 2
done

log ""
log "Done. $NUM_TRIALS CFPA2 trials → $OUT_ROOT"
