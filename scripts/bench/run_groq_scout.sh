#!/usr/bin/env bash
# run_groq_scout.sh — Pri 2: real VLM via Groq llama-4-scout-17b (vision).
#
# Uses the direct-call path added to standalone/vlm/agent.py + explorer.py
# (LangGraph tool-binding fails on Groq with 400 'Failed to call a function').
# Direct path: single-shot LLM call, VLM emits JSON with goal+reason+artifact_seen.
# When artifact_seen=true, explorer synthesizes the artifact-log callback manually.
#
# Output layout:
#   logs/vlm_variations/vlm_groq_scout/
#     ├── sessions/session_*  ← per-cycle scene+map+camera+decision
#     ├── timeseries/         ← coverage_vlm_groq_<ts>.csv
#     ├── reports/            ← vlm_groq_report_<ts>.json
#     └── trial_logs/         ← trial_N_<ts>.log
#
# Usage:
#   ./scripts/bench/run_groq_scout.sh                  # 8 trials default
#   NUM_TRIALS=3 ./scripts/bench/run_groq_scout.sh     # smoke
#   CYCLE_SEC=8 ./scripts/bench/run_groq_scout.sh      # faster cycle
#
# Quota: Groq free tier = 30 RPM, 1000 RPD, 6000 TPM. Direct-call uses 1 HTTP
# call per VLM cycle (no LangGraph multi-call), so 1 trial ≈ 10-20 calls. 8
# trials ≈ 80-160 calls = well within RPD.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NUM_TRIALS="${NUM_TRIALS:-8}"
CYCLE_SEC="${CYCLE_SEC:-12}"
MJPYTHON="${MJPYTHON:-mjpython}"

OUT_ROOT="$REPO_ROOT/logs/vlm_variations/vlm_groq_scout"
mkdir -p "$OUT_ROOT"/{sessions,timeseries,reports,trial_logs}

log() { echo "[groq-scout $(date +%H:%M:%S)] $*"; }

log "==========================================="
log "Pri 2: Groq Scout × $NUM_TRIALS trials (cycle ${CYCLE_SEC}s)"
log "OUT: $OUT_ROOT"
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
        --provider groq \
        --cycle-sec "$CYCLE_SEC" \
        > "$trial_log" 2>&1 || true

    # Move timeseries CSV from default results dir to our dir.
    shopt -s nullglob 2>/dev/null || true
    for f in "$OUT_ROOT/reports"/coverage_*.csv; do
        [ -f "$f" ] || continue
        mv "$f" "$OUT_ROOT/timeseries/" 2>/dev/null || true
    done

    elapsed=$(( $(date +%s) - ts_before ))
    log "[trial $trial] done in ${elapsed}s"
    sleep 3
done

log ""
log "==========================================="
log "Pri 2 DONE. $NUM_TRIALS Groq Scout trials saved to $OUT_ROOT"
log "==========================================="
