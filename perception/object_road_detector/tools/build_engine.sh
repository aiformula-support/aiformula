#!/usr/bin/env bash
# Build the TensorRT engine used by object_road_detector_trt. Engines are specific to this GPU
# and TensorRT version, so they are generated here rather than checked in.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-640}"
python3 "${SCRIPT_DIR}/export_engine.py" --width "${WIDTH}" --height "${HEIGHT}" "$@"
