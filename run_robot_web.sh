#!/bin/sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8088}"

cd "$SCRIPT_DIR"

if command -v hostname >/dev/null 2>&1; then
  PI_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
else
  PI_IP=""
fi

echo "[WEB] Starting robot web UI..."
if [ -n "$PI_IP" ]; then
  echo "[WEB] Control panel: http://$PI_IP:$PORT"
fi

exec python3 "$SCRIPT_DIR/robot_web_ui.py" --host "$HOST" --port "$PORT"
