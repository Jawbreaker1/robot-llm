#!/bin/sh
set -eu

model="${1:-small}"

case "$model" in
    base)
        expected_sha256="60ed5bc3dd14eea856493d334349b405782ddcaf0028d4b5df4088345fba2efe"
        maximum_bytes="200000000"
        ;;
    small)
        expected_sha256="1be3a9b2063867b937e64e2ec7483364a79917e157fa98c5d94b5c1fffea987b"
        maximum_bytes="600000000"
        ;;
    *)
        echo "Usage: sh scripts/download_whisper_model.sh [base|small]" >&2
        exit 2
        ;;
esac

project_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
model_directory="$project_root/models"
destination="$model_directory/ggml-$model.bin"
partial="$destination.part"
url="https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-$model.bin"

checksum() {
    if command -v shasum >/dev/null 2>&1; then
        shasum -a 256 "$1" | awk '{print $1}'
    elif command -v sha256sum >/dev/null 2>&1; then
        sha256sum "$1" | awk '{print $1}'
    else
        echo "A SHA-256 utility is required (shasum or sha256sum)." >&2
        exit 1
    fi
}

mkdir -p "$model_directory"

if [ -f "$destination" ]; then
    actual_sha256="$(checksum "$destination")"
    if [ "$actual_sha256" = "$expected_sha256" ]; then
        echo "Verified model already exists: $destination"
        exit 0
    fi
    echo "Existing model failed checksum verification: $destination" >&2
    exit 1
fi

trap 'rm -f "$partial"' EXIT HUP INT TERM
curl \
    -L \
    --fail \
    --connect-timeout 15 \
    --max-time 1800 \
    --max-filesize "$maximum_bytes" \
    --retry 3 \
    --retry-delay 2 \
    --speed-limit 1024 \
    --speed-time 60 \
    --output "$partial" \
    "$url"

actual_bytes="$(wc -c < "$partial" | tr -d '[:space:]')"
if [ "$actual_bytes" -gt "$maximum_bytes" ]; then
    echo "Downloaded model exceeded its maximum allowed size." >&2
    exit 1
fi

actual_sha256="$(checksum "$partial")"
if [ "$actual_sha256" != "$expected_sha256" ]; then
    echo "Downloaded model failed checksum verification." >&2
    exit 1
fi

mv "$partial" "$destination"
trap - EXIT HUP INT TERM
echo "Downloaded and verified: $destination"
