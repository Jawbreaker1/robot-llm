#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

exec "$project_root/scripts/start_lab_console.sh" \
    --robot-profile ev3rstorm-01 \
    "$@"
