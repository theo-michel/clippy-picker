#!/usr/bin/env bash
# Start the picker: web controller + factory edge (MQTT bridge).
# Usage: ./run.sh [--broker localhost]
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR/python"

cleanup() {
  kill -9 $(jobs -p) 2>/dev/null
  wait 2>/dev/null
}
trap cleanup INT TERM EXIT

uv run python web_controller.py &
sleep 2
uv run python factory_edge.py "$@" &
wait
