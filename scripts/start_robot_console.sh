#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
ev3_target="${ROBOT_LLM_EV3_TARGET-robot@ev3dev.local}"
blast_hub_name="${ROBOT_LLM_BLAST_HUB_NAME-BLAST-01}"

# EV3 is the active goal executor in this combined profile. BLAST is still a
# first-class controller in the dashboard and remains disconnected until the
# operator presses Connect.
exec "$project_root/scripts/start_lab_console.sh" \
    --robot-profile ev3rstorm-01 \
    --robot-target "$ev3_target" \
    --blast-hub-name "$blast_hub_name" \
    "$@"
