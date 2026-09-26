#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
ev3_target="${ROBOT_LLM_EV3_TARGET-robot@ev3dev.local}"
blast_hub_name="${ROBOT_LLM_BLAST_HUB_NAME-BLAST-01}"

# Both robots expose conversation and model-authored navigation. Hardware
# connections and movement begin only when requested by the operator.
exec "$project_root/scripts/start_lab_console.sh" \
    --robot-profile ev3rstorm-01 \
    --robot-target "$ev3_target" \
    --blast-hub-name "$blast_hub_name" \
    "$@"
