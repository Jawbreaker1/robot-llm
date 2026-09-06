#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
model_directory="${ROBOT_LLM_PIPER_MODEL_DIR-$project_root/models/piper}"
port="${ROBOT_LLM_PIPER_PORT-8179}"

for voice in sv_SE-lisa-medium sv_SE-nst-medium; do
    if [ ! -f "$model_directory/$voice.onnx" ] || \
       [ ! -f "$model_directory/$voice.onnx.json" ]; then
        echo "Piper model is missing: $voice" >&2
        echo "Run scripts/setup_piper_service.sh once." >&2
        exit 1
    fi
done

PYTHONPATH="$project_root/src${PYTHONPATH:+:$PYTHONPATH}" \
    exec uvx --from 'piper-tts==1.4.2' \
    python -m robot_agent.piper_sidecar \
    --port "$port" \
    --model-dir "$model_directory"
