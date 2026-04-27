#!/usr/bin/env bash
# Start the picker: web controller + factory edge (MQTT bridge).
#
# Usage:
#   ./run.sh                       # default port 8080
#   ./run.sh --port 8081           # override (web UI + edge agree via PICKER_PORT)
#   ./run.sh --broker localhost    # forwarded to factory_edge.py
#
# If port 8080 is held by a stale web_controller that won't die on `kill -9`,
# it's blocked inside a USB-serial ioctl. Either unplug the ESP32 USB for 5 s,
# or just start on another port with --port.
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR/python"

PORT="${PICKER_PORT:-8080}"
FE_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --port)   PORT="$2"; shift 2 ;;
    --port=*) PORT="${1#--port=}"; shift ;;
    *)        FE_ARGS+=("$1"); shift ;;
  esac
done

port_busy() { [ -n "$(lsof -t -iTCP:"$1" -sTCP:LISTEN 2>/dev/null)" ]; }

if port_busy "$PORT"; then
  orig="$PORT"
  for alt in 8081 8082 8083 8090 9080; do
    if ! port_busy "$alt"; then PORT="$alt"; break; fi
  done
  if [ "$PORT" = "$orig" ]; then
    echo "Port $orig is in use and no fallback (8081-8083, 8090, 9080) is free." >&2
    exit 1
  fi
  echo "Port $orig is busy; falling back to $PORT." >&2
fi

cleanup() {
  kill -9 $(jobs -p) 2>/dev/null
  wait 2>/dev/null
}
trap cleanup INT TERM EXIT

export PICKER_PORT="$PORT"
echo "Picker on http://localhost:${PICKER_PORT}" >&2

uv run python web_controller.py &
sleep 2
uv run python factory_edge.py "${FE_ARGS[@]}" &
wait
