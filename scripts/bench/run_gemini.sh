#!/usr/bin/env bash
# run_gemini.sh — Pri 3: real VLM via Google Gemini 2.5-Flash-Lite.
# Quota-bound: ~20 RPD per model. Default 2 trials.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM_TRIALS="${NUM_TRIALS:-2}"
CYCLE_SEC="${CYCLE_SEC:-15}"
MJPYTHON="${MJPYTHON:-mjpython}"

OUT_ROOT="$REPO_ROOT/logs/vlm_variations/vlm_google"
mkdir -p "$OUT_ROOT"/{sessions,timeseries,reports,trial_logs}

log() { echo "[gemini $(date +%H:%M:%S)] $*"; }

log "==========================================="
log "Pri 3: Gemini × $NUM_TRIALS trials (cycle ${CYCLE_SEC}s)"
log "==========================================="

for trial in $(seq 1 "$NUM_TRIALS"); do
    ts=$(date +%Y%m%d_%H%M%S)
    trial_log="$OUT_ROOT/trial_logs/trial${trial}_${ts}.log"
    ts_before=$(date +%s)
    log "[trial $trial/$NUM_TRIALS] starting → $trial_log"
    STANDALONE_VLM_LOG="$OUT_ROOT/sessions" \
    STANDALONE_RESULTS_DIR="$OUT_ROOT/reports" \
    "$MJPYTHON" "$REPO_ROOT/standalone/main_vlm.py" \
        --headless --no-map \
        --provider google \
        --cycle-sec "$CYCLE_SEC" \
        > "$trial_log" 2>&1 || true
    shopt -s nullglob 2>/dev/null || true
    for f in "$OUT_ROOT/reports"/coverage_*.csv; do
        [ -f "$f" ] || continue
        mv "$f" "$OUT_ROOT/timeseries/" 2>/dev/null || true
    done
    elapsed=$(( $(date +%s) - ts_before ))
    log "[trial $trial] done in ${elapsed}s"
    sleep 5
done

log "==========================================="
log "Pri 3 DONE. $NUM_TRIALS Gemini trials → $OUT_ROOT"
log "==========================================="
