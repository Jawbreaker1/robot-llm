#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$project_root"

export PYTHONDONTWRITEBYTECODE=1
export PYTHONWARNINGS=error
export PYTHONPATH="$project_root/src${PYTHONPATH:+:$PYTHONPATH}"

node --check src/robot_agent/dashboard_web/i18n.js
node --check src/robot_agent/dashboard_web/dashboard_logic.js
node --check src/robot_agent/dashboard_web/spatial_map_presenter.js
node --check src/robot_agent/dashboard_web/speech_input_logic.js
node --check src/robot_agent/dashboard_web/microphone_input.js
node --check src/robot_agent/dashboard_web/pcm_capture_worklet.js
node --check src/robot_agent/dashboard_web/app.js
python3 -m unittest discover -s tests -q
