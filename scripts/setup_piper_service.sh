#!/bin/sh
set -eu

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
model_directory="$project_root/models/piper"

mkdir -p "$model_directory"
exec uvx --from 'piper-tts==1.4.2' \
    python -m piper.download_voices \
    --data-dir "$model_directory" \
    sv_SE-lisa-medium \
    sv_SE-nst-medium
