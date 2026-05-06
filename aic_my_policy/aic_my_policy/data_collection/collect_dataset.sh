#!/usr/bin/env bash
# Outer orchestrator for synthetic dataset collection.
#
# For each sample: generate a randomized launch-arg set, start
# `aic_bringup`, wait for settle, invoke `capture_scene`, shut down.
#
# Usage:
#   bash collect_dataset.sh <output_dir> <num_samples> [<start_idx>] [random|sfp|sc]
#
# Run this *outside* any running aic_model; it owns the bringup session.

set -u

OUT_DIR="${1:?output dir required}"
N="${2:?num samples required}"
START_IDX="${3:-0}"
TRIAL_KIND="${4:-random}"
SETTLE_SEC="${SETTLE_SEC:-3.0}"
LAUNCH_READY_SEC="${LAUNCH_READY_SEC:-20.0}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
ROS2_BIN="${ROS2_BIN:-ros2}"
POLICY_MODULE_PREFIX="${POLICY_MODULE_PREFIX:-aic_my_policy.aic_my_policy}"

mkdir -p "$OUT_DIR"

for (( i = START_IDX; i < START_IDX + N; i++ )); do
    SAMPLE_PATH="$OUT_DIR/sample_$(printf '%06d' "$i").npz"
    if [[ -f "$SAMPLE_PATH" ]]; then
        echo "[$i] exists, skipping."
        continue
    fi

    # Dump randomized launch args via the Python helper.
    ARGS_FILE="$(mktemp)"
    "$PYTHON_BIN" -c "
import json, sys
from ${POLICY_MODULE_PREFIX}.data_collection.randomize import sample_scene_config
cfg = sample_scene_config(seed=$i, trial_kind='$TRIAL_KIND')
print(' '.join(cfg.as_launch_args()))
" > "$ARGS_FILE" || { echo "randomize failed"; exit 1; }
    LAUNCH_ARGS="$(cat "$ARGS_FILE")"
    rm -f "$ARGS_FILE"

    echo "[$i] launching: $LAUNCH_ARGS"
    setsid "$ROS2_BIN" launch aic_bringup aic_gz_bringup.launch.py $LAUNCH_ARGS > /tmp/aic_launch_$i.log 2>&1 &
    LAUNCH_PID=$!
    sleep "$LAUNCH_READY_SEC"

    if ! ps -p "$LAUNCH_PID" > /dev/null; then
        echo "[$i] launch died early; see /tmp/aic_launch_$i.log"
        continue
    fi

    "$PYTHON_BIN" -m "${POLICY_MODULE_PREFIX}.data_collection.capture_scene" \
        --out "$SAMPLE_PATH" --settle "$SETTLE_SEC" \
        || echo "[$i] capture failed"

    kill -INT "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 5 
    kill -KILL "-$LAUNCH_PID" 2>/dev/null  || true

    wait "$LAUNCH_PID" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "ruby.*gz" 2>/dev/null || true
    sleep 2
done

echo "Done. Samples in $OUT_DIR"
