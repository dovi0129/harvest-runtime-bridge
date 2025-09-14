#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python -m apps.fake_detection_sender
