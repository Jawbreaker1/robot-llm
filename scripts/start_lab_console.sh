#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
if [ -n "${ROBOT_LLM_PYTHON-}" ]; then
    python_command="$ROBOT_LLM_PYTHON"
elif [ -x "$project_root/.venv/bin/python" ]; then
    python_command="$project_root/.venv/bin/python"
else
    python_command="python3"
fi
stt_url="${ROBOT_LLM_STT_URL-http://127.0.0.1:8178/v1}"
stt_inference_path="${ROBOT_LLM_STT_INFERENCE_PATH-/audio/transcriptions}"
stt_model_id="${ROBOT_LLM_STT_MODEL_ID-ggml-large-v3-turbo-q5_0}"
console_access_key_file="${ROBOT_LLM_CONSOLE_ACCESS_KEY_FILE-~/.robot-llm/dashboard-access-key}"

set -- \
    --console-access-key-file "$console_access_key_file" \
    "$@"

if [ -n "$stt_url" ]; then
    set -- \
        --stt-url "$stt_url" \
        --stt-inference-path "$stt_inference_path" \
        --stt-model-id "$stt_model_id" \
        "$@"
fi

if [ -n "${PYTHONPATH-}" ]; then
    PYTHONPATH="$project_root/src:$PYTHONPATH"
else
    PYTHONPATH="$project_root/src"
fi
export PYTHONPATH

exec "$python_command" -m robot_agent.dashboard_cli "$@"
