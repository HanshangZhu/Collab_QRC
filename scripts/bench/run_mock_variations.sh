#!/usr/bin/env bash
# run_mock_variations.sh — Pri 1: 5 trials × 3 mock strategies on demo3.
#
# Strategies:
#   - random         → main_random.py (Yamauchi 1997 random frontier)
#   - greedy_nearest → main_greedy.py (Yamauchi 1997 nearest frontier)
#   - info_gain      → main_vlm.py --provider mock --strategy info_gain
#                       (Bircher 2016 NBV spirit; CFPA2 utility w/o A* preval)
#
# Each trial runs in headless mode until coverage stagnates or unreachable
# threshold trips. Saves per-trial: session dir (vlm cycles), timeseries CSV,
# end-of-run JSON report, full stdout.
#
# Output layout:
#   logs/vlm_variations/mock_<strategy>/
#     ├── sessions/session_*  ← per-cycle scene + map + decision (mock = trivial)
#     ├── timeseries/         ← coverage_<strategy>_<ts>.csv
#     ├── reports/            ← vlm_report_<ts>.json
#     └── trial_logs/         ← trial_N_<ts>.log
#
# Usage:
#   ./scripts/bench/run_mock_variations.sh
#   NUM_TRIALS=3 ./scripts/bench/run_mock_variations.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM_TRIALS="${NUM_TRIALS:-5}"
MJPYTHON="${MJPYTHON:-mjpython}"

log() { echo "[mock-bench $(date +%H:%M:%S)] $*"; }

run_one() {
    local strategy="$1"
    local script="$2"
    local extra_flags="$3"
    local trial="$4"

    local out_root="$REPO_ROOT/logs/vlm_variations/mock_${strategy}"
    mkdir -p "$out_root"/{sessions,timeseries,reports,trial_logs}

    local ts; ts=$(date +%Y%m%d_%H%M%S)
    local trial_log="$out_root/trial_logs/trial${trial}_${ts}.log"
    local ts_before; ts_before=$(date +%s)

    log "[$strategy] trial $trial/$NUM_TRIALS starting → $trial_log"

    # Direct each trial's session/timeseries to the strategy-specific dir.
    STANDALONE_VLM_LOG="$out_root/sessions" \
    STANDALONE_RESULTS_DIR="$out_root/timeseries" \
    "$MJPYTHON" "$REPO_ROOT/standalone/$script" \
        --headless --no-map $extra_flags \
        > "$trial_log" 2>&1 || true

    # Copy end-of-run report from /tmp (main_vlm.py writes there)
    for f in /tmp/vlm_report_*.json; do
        [ -f "$f" ] || continue
        local file_ts; file_ts=$(stat -f "%m" "$f" 2>/dev/null || stat -c "%Y" "$f" 2>/dev/null || echo 0)
        [ "$file_ts" -ge "$ts_before" ] || continue
        cp "$f" "$out_root/reports/trial${trial}_$(basename "$f")"
    done

    local elapsed=$(( $(date +%s) - ts_before ))
    log "[$strategy] trial $trial done in ${elapsed}s"
    sleep 2
}

# Map strategy → (script, extra_flags)
declare -a STRATEGIES=("random" "greedy_nearest" "info_gain")
declare -a SCRIPTS=("main_random.py" "main_greedy.py" "main_vlm.py")
declare -a EXTRAS=("" "" "--provider mock --strategy info_gain --cycle-sec 2.0")

log "==========================================="
log "Mock variation bench: $NUM_TRIALS trials × 3 strategies = $((NUM_TRIALS * 3)) trials"
log "==========================================="

for i in "${!STRATEGIES[@]}"; do
    strategy="${STRATEGIES[$i]}"
    script="${SCRIPTS[$i]}"
    extra="${EXTRAS[$i]}"
    log ""
    log "=== STRATEGY: $strategy ==="
    for trial in $(seq 1 "$NUM_TRIALS"); do
        run_one "$strategy" "$script" "$extra" "$trial"
    done
done

log ""
log "==========================================="
log "Mock variation bench DONE."
log "==========================================="
