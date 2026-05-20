#!/usr/bin/env bash
# run_ablations.sh — Pri 4/6/7/8: targeted ablations on Groq Scout.
#
# Conditions (all use Groq Scout direct-call path):
#   Pri 4: --no-info-gain × 2 trials    (strip info_gain from scene JSON)
#   Pri 6: --no-camera    × 2 trials    (no camera image in VLM input)
#   Pri 7: --no-history   × 2 trials    (no exploration history in prompt)
#   Pri 8: --cycle-sec 6  × 2 trials    (faster cycle)
#   Pri 8: --cycle-sec 24 × 2 trials    (slower cycle)
#
# Total: 10 trials × ~3 min = ~30 min wall-clock.
# Quota: 10 × ~10 cycles × 1 call = 100 Groq calls (≤10% of 1000 RPD).
#
# Output:
#   logs/vlm_variations/vlm_groq_<cond>/{sessions,timeseries,reports,trial_logs}/
#
# Usage:
#   ./scripts/bench/run_ablations.sh
#   NUM_TRIALS=1 ./scripts/bench/run_ablations.sh    # smoke (5 trials total)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM_TRIALS="${NUM_TRIALS:-2}"
MJPYTHON="${MJPYTHON:-mjpython}"

log() { echo "[ablate $(date +%H:%M:%S)] $*"; }

run_cond() {
    local cond_name="$1"
    local cycle_sec="$2"
    shift 2
    # Remaining args = extra flags to main_vlm.py

    local out_root="$REPO_ROOT/logs/vlm_variations/vlm_groq_${cond_name}"
    mkdir -p "$out_root"/{sessions,timeseries,reports,trial_logs}

    for trial in $(seq 1 "$NUM_TRIALS"); do
        local ts; ts=$(date +%Y%m%d_%H%M%S)
        local trial_log="$out_root/trial_logs/trial${trial}_${ts}.log"
        local ts_before; ts_before=$(date +%s)

        log "[$cond_name] trial $trial/$NUM_TRIALS starting"

        STANDALONE_VLM_LOG="$out_root/sessions" \
        STANDALONE_RESULTS_DIR="$out_root/reports" \
        "$MJPYTHON" "$REPO_ROOT/standalone/main_vlm.py" \
            --headless --no-map \
            --provider groq \
            --cycle-sec "$cycle_sec" \
            "$@" \
            > "$trial_log" 2>&1 || true

        shopt -s nullglob 2>/dev/null || true
        for f in "$out_root/reports"/coverage_*.csv; do
            [ -f "$f" ] || continue
            mv "$f" "$out_root/timeseries/" 2>/dev/null || true
        done

        local elapsed=$(( $(date +%s) - ts_before ))
        log "[$cond_name] trial $trial done in ${elapsed}s"
        sleep 2
    done
}

log "==========================================="
log "Pri 4/6/7/8: ablations × $NUM_TRIALS trials each (5 conditions)"
log "==========================================="

# Pri 4: strip info_gain
log ""; log "=== Pri 4: no-info-gain (cycle=12) ==="
run_cond "noig"  12 --no-info-gain

# Pri 6: no camera
log ""; log "=== Pri 6: no-camera (cycle=12) ==="
run_cond "nocam" 12 --no-camera

# Pri 7: no history
log ""; log "=== Pri 7: no-history (cycle=12) ==="
run_cond "nohist" 12 --no-history

# Pri 8a: fast cycle
log ""; log "=== Pri 8a: cycle=6s ==="
run_cond "cyc6"  6

# Pri 8b: slow cycle
log ""; log "=== Pri 8b: cycle=24s ==="
run_cond "cyc24" 24

log ""
log "==========================================="
log "Ablations DONE. 5 conditions × $NUM_TRIALS trials each."
log "==========================================="
