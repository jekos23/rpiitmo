#!/bin/sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.robot_web_ui.env"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8088}"
PYTHON_BIN="${PYTHON_BIN:-}"
VENV_PATH="${VENV_PATH:-}"

cd "$SCRIPT_DIR"

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi

if [ -z "$PYTHON_BIN" ]; then
  if [ -n "$VENV_PATH" ] && [ -x "$VENV_PATH/bin/python" ]; then
    PYTHON_BIN="$VENV_PATH/bin/python"
  else
    for candidate in \
      "$SCRIPT_DIR/mienv" \
      "$SCRIPT_DIR/.mienv" \
      "$SCRIPT_DIR/.venv" \
      "$SCRIPT_DIR/venv" \
      "$SCRIPT_DIR/../.venv" \
      "$SCRIPT_DIR/../venv"
    do
      if [ -x "$candidate/bin/python" ]; then
        PYTHON_BIN="$candidate/bin/python"
        break
      fi
    done
  fi
fi

if [ -z "$PYTHON_BIN" ]; then
  PYTHON_BIN="$(command -v python3 || true)"
fi

if [ -z "$PYTHON_BIN" ]; then
  echo "[WEB] python3 was not found and no venv interpreter was detected."
  echo "[WEB] Set VENV_PATH or PYTHON_BIN before starting the web UI."
  exit 1
fi

echo "[WEB] Python: $PYTHON_BIN"

if command -v hostname >/dev/null 2>&1; then
  PI_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
else
  PI_IP=""
fi

echo "[WEB] Starting robot web UI..."
if [ -n "$PI_IP" ]; then
  echo "[WEB] Control panel: http://$PI_IP:$PORT"
fi

exec "$PYTHON_BIN" "$SCRIPT_DIR/robot_web_ui.py" --host "$HOST" --port "$PORT"
