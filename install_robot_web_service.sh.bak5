#!/bin/sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SERVICE_NAME="robot-web-ui.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
ENV_FILE="$SCRIPT_DIR/.robot_web_ui.env"
ENV_HOST=""
ENV_PORT=""
ENV_VENV_PATH=""

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
  ENV_HOST="${HOST:-}"
  ENV_PORT="${PORT:-}"
  ENV_VENV_PATH="${VENV_PATH:-}"
fi

WEB_HOST="${WEB_HOST:-${ENV_HOST:-0.0.0.0}}"
WEB_PORT="${WEB_PORT:-${ENV_PORT:-8088}}"
VENV_PATH="${VENV_PATH:-$ENV_VENV_PATH}"

chmod +x "$SCRIPT_DIR/run_robot_web.sh"

cat > "$ENV_FILE" <<EOF
HOST="$WEB_HOST"
PORT="$WEB_PORT"
VENV_PATH="$VENV_PATH"
EOF

chmod 600 "$ENV_FILE"

cat <<EOF | sudo tee "$SERVICE_PATH" >/dev/null
[Unit]
Description=Robot Web Control UI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$SCRIPT_DIR
EnvironmentFile=$ENV_FILE
ExecStart=$SCRIPT_DIR/run_robot_web.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE_NAME"
echo "[SERVICE] Installed and started $SERVICE_NAME"
echo "[SERVICE] Open http://<RASPBERRY_PI_IP>:$WEB_PORT"
