#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
blast_hub_name="${ROBOT_LLM_BLAST_HUB_NAME-BLAST-01}"

exec "$project_root/scripts/start_lab_console.sh" \
    --robot-profile blast-01 \
    --blast-hub-name "$blast_hub_name" \
    "$@"
