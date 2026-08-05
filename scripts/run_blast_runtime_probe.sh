#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
python_command="${ROBOT_LLM_PYTHON-$project_root/.venv/bin/python}"

if [ ! -x "$python_command" ]; then
    echo "Python environment not found: $python_command" >&2
    echo "Create .venv and install requirements-pybricks.txt first." >&2
    exit 2
fi

if [ -n "${PYTHONPATH-}" ]; then
    PYTHONPATH="$project_root/src:$PYTHONPATH"
else
    PYTHONPATH="$project_root/src"
fi
export PYTHONPATH

exec "$python_command" -m robot_agent.blast_ble_runtime "$@"
