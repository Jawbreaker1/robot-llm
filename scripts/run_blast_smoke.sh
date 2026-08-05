#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
python_command="${ROBOT_LLM_PYTHON-$project_root/.venv/bin/python}"
hub_name="${BLAST_HUB_NAME-BLAST-01}"

if [ ! -x "$python_command" ]; then
    echo "Python environment not found: $python_command" >&2
    echo "Create .venv and install requirements-pybricks.txt first." >&2
    exit 2
fi

exec "$python_command" -m pybricksdev run ble \
    --name "$hub_name" \
    "$project_root/hub_programs/blast_01/smoke.py"
